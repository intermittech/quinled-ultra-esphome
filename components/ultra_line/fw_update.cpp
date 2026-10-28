// "Update all receivers on this port": the update entity (fw_update.h).
#include "esphome/core/defines.h"
#include "fw_update.h"
#include "ultra_text.h"

#if defined(USE_ESP32) && defined(USE_ULTRA_LINE_UPDATE)

#include "esphome/components/json/json_util.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <algorithm>
#include <cinttypes>
#include <memory>
#include <esp_random.h>
#include <esp_timer.h>

namespace esphome::ultra_line {

static const char *const TAG = "ultra_line.update";

UltraFirmwareUpdate *UltraFirmwareUpdate::active_ = nullptr;

static constexpr uint32_t INITIAL_CHECK_ID = 0x44324146;  // set_interval id: the first check once the network is up
static constexpr uint32_t INITIAL_CHECK_MS = 10000;
static constexpr uint32_t PLAN_MS = 10000;               // the plan again this often (the receivers' identities)
static constexpr size_t MANIFEST_MAX = 16384;
static constexpr size_t SIG_MAX = 160;                   // DER ECDSA P-256: 70-72 bytes
static constexpr size_t SUMMARY_MAX = 255;               // Home Assistant's release summary

// Inputs copied on the main loop (the task never reads members that the main loop may change), outputs for defer().
struct UltraFirmwareUpdate::CheckResult {
  UltraFirmwareUpdate *self{nullptr};
  uint8_t channel{0};
  std::string url;
  FwManifest manifest;
  std::string error;
  const char *key{nullptr};
};

// ------------------------------------------------------------------------------------------------ setup / config
void UltraFirmwareUpdate::setup() {
  this->hub_->set_recover_handler([this](uint16_t board) { this->recover(board); });   // (its receivers' buttons)
  this->report_pref_ = this->make_entity_preference<PortReportStore>();
  this->ident_pref_ = this->make_entity_preference<PortIdentityStore>(PortIdentityStore::MAGIC);
  if (!this->ident_pref_.load(&this->idents_))
    this->idents_ = PortIdentityStore{};
  auto store = std::make_unique<PortReportStore>();
  this->have_report_ = this->report_pref_.load(store.get()) && port_report_load(*store, this->report_);
  if (this->have_report_)
    this->set_report_text_(port_report_text(this->report_));
  this->update_info_.title = "QuinLED Ultra receivers";
  this->update_info_.current_version = "unknown";
  this->publish_state();
  // without signing keys (public_keys: production before QuinLED provisions them) nothing can install: say so
  this->set_status_(this->verify_ && this->nkeys_ == 0
                        ? "no signing keys in this build: no release can be checked yet (QuinLED provisions the "
                          "production keys with the firmware's first release)"
                        : "idle");
  // the first look once the network is up, then every update_interval
  if (this->get_update_interval() != SCHEDULER_DONT_RUN) {
    this->set_interval(INITIAL_CHECK_ID, INITIAL_CHECK_MS, [this]() {
      if (!network::is_connected())
        return;
      this->cancel_interval(INITIAL_CHECK_ID);
      this->update();
    });
  }
}

void UltraFirmwareUpdate::dump_config() {
  const uint8_t port = this->hub_->port_type();
  ESP_LOGCONFIG(TAG,
                "QuinLED Ultra port update '%s':\n"
                "  Channel: %s, manifest %s\n"
                "  Signature: %s (%u public keys)\n"
                "  Port: %s, %u boards (at most %u); provisioned positions: %u",
                this->get_name().c_str(), fw_channel_name(this->channel_),
                fw_manifest_url(this->source_, this->channel_).c_str(), this->verify_ ? "checked" : "NOT checked",
                static_cast<unsigned>(this->nkeys_),
                port == PT_DIFF_POWER ? "Diff-Power" : port == PT_D2A_PIN ? "Dig2Analog-Plus-5ch pin" : "?",
                this->hub_->get_num_boards(), port_max_boards(port),
                static_cast<unsigned>(this->provisions_.size()));
  if (this->verify_ && this->nkeys_ == 0)
    ESP_LOGW(TAG, "  No signing keys in this build (QuinLED provisions the production keys with the firmware's first "
                  "release; bench builds: public_keys: test with your own source)");
}

bool UltraFirmwareUpdate::add_inventory(uint16_t board, const std::vector<uint8_t> &boot_info,
                                        const std::vector<uint8_t> &uid, bool return_channel, bool has_layout,
                                        uint32_t metadata_base, uint32_t metadata_bytes, uint32_t bootloader_bytes,
                                        bool has_blackout, uint32_t blackout_ms) {
  if (boot_info.size() != UPD_BL_INFO_BYTES || (!uid.empty() && uid.size() != 12) ||
      !port_boot_info_valid(boot_info.data()))
    return false;
  PortProvision p;
  p.position = board;
  std::memcpy(p.boot_info, boot_info.data(), UPD_BL_INFO_BYTES);
  p.has_uid = uid.size() == 12;
  if (p.has_uid)
    std::memcpy(p.uid, uid.data(), 12);
  p.return_channel = return_channel && p.has_uid;
  p.has_layout = has_layout;
  p.metadata_base = metadata_base;
  p.metadata_bytes = metadata_bytes;
  p.bootloader_bytes = bootloader_bytes;
  p.has_blackout = has_blackout;
  p.blackout_ms = blackout_ms;
  this->provisions_.push_back(p);
  return true;
}

void UltraFirmwareUpdate::set_status_(const std::string &status) {
  const std::string text = text_state(status);   // (Home Assistant takes up to 255 characters)
  if (text == this->status_text_)
    return;
  this->status_text_ = text;
  ESP_LOGD(TAG, "'%s': %s", this->get_name().c_str(), text.c_str());
#ifdef USE_TEXT_SENSOR
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(text);
#endif
}

void UltraFirmwareUpdate::set_report_text_(const std::string &text) {
  if (text == this->report_text_)
    return;
  this->report_text_ = text;
#ifdef USE_TEXT_SENSOR
  if (this->report_sensor_ != nullptr)
    this->report_sensor_->publish_state(text);
#endif
}

void UltraFirmwareUpdate::change_channel(uint8_t channel, bool check) {
  if (fw_channel_name(channel) == nullptr || channel == this->channel_)
    return;
  this->channel_ = channel;
  this->have_release_ = false;
  this->update_info_.latest_version.clear();
  if (!check)
    return;
  ESP_LOGI(TAG, "'%s': channel %s", this->get_name().c_str(), fw_channel_name(channel));
  this->state_ = update::UPDATE_STATE_UNKNOWN;
  this->publish_state();
  this->update();
}

// ------------------------------------------------------------------------------------------------ check
void UltraFirmwareUpdate::update() {
  if (this->check_task_handle_ != nullptr)
    return;  // one already runs
  this->log_report_();   // (Check for updates: the last job's full report in the log again)
  if (!network::is_connected()) {
    ESP_LOGD(TAG, "Network not connected: no check");
    return;
  }
  auto *r = new CheckResult();
  r->self = this;
  r->channel = this->channel_;
  r->url = fw_manifest_url(this->source_, this->channel_);
  if (xTaskCreate(UltraFirmwareUpdate::check_task_, "d2a_fw_check", 8192, r, 1, &this->check_task_handle_) != pdPASS) {
    this->check_task_handle_ = nullptr;
    delete r;
    ESP_LOGW(TAG, "Cannot start the check task");
  }
}

bool UltraFirmwareUpdate::fetch_(const std::string &url, std::vector<uint8_t> &out, size_t max_len) {
  auto c = this->http_->get(url);
  if (c == nullptr)
    return false;
  bool ok = false;
  if (c->status_code == http_request::HTTP_STATUS_OK && c->content_length > 0 && c->content_length <= max_len) {
    out.resize(c->content_length);
    const auto rr = http_request::http_read_fully(c.get(), out.data(), out.size(), 512, this->http_->get_timeout());
    ok = rr.status == http_request::HttpReadStatus::OK && c->get_bytes_read() == out.size();
  } else if (c->status_code == http_request::HTTP_STATUS_OK && c->content_length == 0) {
    // no Content-Length (a chunked response, e.g. from a CDN): read it in pieces, at most max_len bytes
    out.clear();
    uint8_t chunk[512];
    uint32_t last = millis();
    for (;;) {
      const int got = c->read(chunk, sizeof(chunk));
      const auto result = http_request::http_read_loop_result(got, last, this->http_->get_timeout(),
                                                              c->is_read_complete());
      if (result == http_request::HttpReadLoopResult::RETRY)
        continue;
      if (result == http_request::HttpReadLoopResult::COMPLETE) {
        ok = !out.empty();
        break;
      }
      if (result != http_request::HttpReadLoopResult::DATA || static_cast<size_t>(got) > max_len - out.size())
        break;
      out.insert(out.end(), chunk, chunk + got);
    }
  } else {
    ESP_LOGD(TAG, "%s: HTTP %d, %u bytes", url.c_str(), c->status_code, static_cast<unsigned>(c->content_length));
  }
  c->end();
  return ok;
}

void UltraFirmwareUpdate::check_task_(void *arg) {
  auto *r = static_cast<CheckResult *>(arg);
  UltraFirmwareUpdate *self = r->self;
  std::vector<uint8_t> body, sig;
  if (r->url.empty()) {
    r->error = "no manifest URL";
  } else if (!self->fetch_(r->url, body, MANIFEST_MAX)) {
    r->error = "cannot fetch " + r->url;
  } else if (self->verify_ && !self->fetch_(r->url + ".sig", sig, SIG_MAX)) {
    r->error = "cannot fetch " + r->url + ".sig";
  } else if (self->verify_ && !fw_verify_signature(body.data(), body.size(), sig.data(), sig.size(), r->channel,
                                                   self->keys_, self->nkeys_, &r->key)) {
    r->error = self->nkeys_ == 0 ? "no public keys in this build" : "the manifest's signature does not verify";
  } else {
    FwManifest &m = r->manifest;
    const bool parsed = json::parse_json(body.data(), body.size(), [&m](JsonObject root) -> bool {
      if (!root["version"].is<const char *>() || !root["channel"].is<const char *>() || !root["images"].is<JsonArray>())
        return false;
      m.name = root["name"] | "QuinLED Ultra receivers";
      m.channel = root["channel"].as<std::string>();
      m.version = root["version"].as<std::string>();
      m.release_url = root["release_url"] | "";
      m.summary = root["summary"] | "";
      for (JsonObject img : root["images"].as<JsonArray>()) {
        FwImageInfo info;
        if (!img["target"].is<const char *>() ||
            !fw_hex_to_bytes(img["target"].as<std::string>(), info.target.bytes, 16) ||
            !img["rev_min"].is<unsigned>() || img["rev_min"].as<unsigned>() > 255 ||
            !img["rev_max"].is<unsigned>() || img["rev_max"].as<unsigned>() > 255 ||
            !img["min_bootloader"].is<unsigned>() || img["min_bootloader"].as<unsigned>() > 255)
          return false;
        info.rev_min = img["rev_min"].as<unsigned>();
        info.rev_max = img["rev_max"].as<unsigned>();
        info.min_bootloader = img["min_bootloader"].as<unsigned>();
        info.size = img["size"] | 0;
        info.sha256 = img["sha256"] | "";
        info.url = img["url"] | "";
        info.version = img["version"] | "";
        if (!img["descriptor"].is<const char *>() ||
            !fw_hex_to_bytes(img["descriptor"].as<std::string>(), info.descriptor, UPD_DESC_BYTES))
          return false;
        m.images.push_back(info);
      }
      // the trusted facts of the targets the YAML inventory doesn't provision, by their full 16-byte target
      if (root["layouts"].is<JsonArray>()) {
        for (JsonObject lay : root["layouts"].as<JsonArray>()) {
          FwLayout l;
          if (!lay["target"].is<const char *>() || !fw_hex_to_bytes(lay["target"].as<std::string>(), l.target.bytes, 16) ||
              !lay["metadata_base"].is<unsigned>() || !lay["metadata_bytes"].is<unsigned>() ||
              !lay["bootloader_bytes"].is<unsigned>() || !lay["blackout_ms"].is<unsigned>())
            return false;
          l.metadata_base = lay["metadata_base"].as<uint32_t>();
          l.metadata_bytes = lay["metadata_bytes"].as<uint32_t>();
          l.bootloader_bytes = lay["bootloader_bytes"].as<uint32_t>();
          l.blackout_ms = lay["blackout_ms"].as<uint32_t>();
          l.gap_us = static_cast<uint16_t>(std::min<unsigned>(lay["gap_us"] | 0u, 65535u));
          m.layouts.push_back(l);
        }
      }
      return true;
    });
    const char *err = parsed ? fw_check_manifest(m, r->channel) : "the manifest is not valid JSON";
    if (err != nullptr)
      r->error = err;
  }
  self->defer([self, r]() {
    self->on_checked_(r);
    delete r;
  });
  vTaskDelete(nullptr);
}

void UltraFirmwareUpdate::on_checked_(CheckResult *r) {
  this->check_task_handle_ = nullptr;
  if (r->channel != this->channel_) {
    this->update();  // the channel changed meanwhile: look at the new one
    return;
  }
  if (!r->error.empty()) {
    ESP_LOGW(TAG, "'%s': check failed: %s", this->get_name().c_str(), r->error.c_str());
    this->status_set_warning();
    this->set_status_("check failed: " + r->error);
    return;
  }
  this->status_clear_warning();
  ESP_LOGI(TAG, "'%s': %s %s (signed by the %s): %u images, %u layouts", this->get_name().c_str(),
           fw_channel_name(r->channel), r->manifest.version.c_str(), r->key ? r->key : "no key (unchecked)",
           static_cast<unsigned>(r->manifest.images.size()), static_cast<unsigned>(r->manifest.layouts.size()));
  const bool was = this->have_release_ && this->state_ == update::UPDATE_STATE_AVAILABLE;
  this->have_release_ = true;
  this->manifest_ = std::move(r->manifest);
  this->manifest_url_ = r->url;
  this->update_info_.title = this->manifest_.name;
  this->update_info_.latest_version = this->manifest_.version;
  this->update_info_.release_url = this->manifest_.release_url;
  this->update_info_.firmware_url = "";
  this->replan_();
  if (!was && this->state_ == update::UPDATE_STATE_AVAILABLE)
    this->get_update_available_trigger()->trigger(this->update_info_);
}

// ------------------------------------------------------------------------------------------------ the plan
PortInventory UltraFirmwareUpdate::inventory_(uint32_t now_ms) const {
  std::vector<PortObservation> obs;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->hub_->has_telemetry())
    obs = port_observe(this->hub_->telemetry(), now_ms);
#endif
  return build_port_inventory(this->hub_->get_num_boards(), this->provisions_.data(), this->provisions_.size(),
                              obs.data(), obs.size(), now_ms, &this->idents_);
}

// The receivers' live identities into the ledger kept on the ESP32 (saved only when one changed)
void UltraFirmwareUpdate::remember_(const PortInventory &inv) {
  this->remember_ms_ = millis();
  uint8_t replaced = 0;
  if (port_remember(this->idents_, inv, &replaced))
    this->ident_pref_.save(&this->idents_);
  for (uint16_t k = 0; k < PortIdentityStore::NODES; k++) {
    if (replaced & (1u << k))
      ESP_LOGI(TAG, "'%s': B%u answers with another UID than the board recorded there: its record replaced",
               this->get_name().c_str(), static_cast<unsigned>(k) + 1);
  }
}

// The ledger on the flash before a job: a save is only queued (the preferences write it at their
// next flush, 60 s by default), and a power cut after a board revoked its app and before that flush would lose the
// identity its recovery needs. Saved, written now (sync writes only blobs that changed: at most one write per job),
// read back and compared. Before the job takes the line: no update frame waits for it.
bool UltraFirmwareUpdate::persist_idents_() {
  PortIdentityStore stored{};
  return this->ident_pref_.save(&this->idents_) && global_preferences->sync() && this->ident_pref_.load(&stored) &&
         std::memcmp(&stored, &this->idents_, sizeof(stored)) == 0;
}

// The lowest version a node with the release's image runs ("mixed" when they differ); one-way nodes are never known
std::string UltraFirmwareUpdate::installed_text_() const {
  uint32_t low = 0, high = 0;
  bool any = false;
  for (const auto &p : this->plan_.nodes) {
    if (!p.before_known || p.expected == 0)
      continue;
    low = any ? std::min(low, p.before) : p.before;
    high = any ? std::max(high, p.before) : p.before;
    any = true;
  }
  if (!any)
    return "unknown";
  return format_fw_version(low) + (low != high ? " (mixed)" : "");
}

void UltraFirmwareUpdate::replan_() {
  const uint32_t now = millis();
  this->plan_ms_ = now;
  this->inv_ = this->inventory_(now);
  this->remember_(this->inv_);
  this->plan_ = build_port_plan(this->inv_, this->manifest_, this->hub_->port_type());
  const PortReport planned = port_report_from_plan(this->plan_, &this->inv_);
  bool needed = false;
  for (const auto &r : planned.nodes)
    needed |= port_needs_update(r, this->have_report_ ? &this->report_ : nullptr);
  std::string summary = port_plan_summary(this->plan_);
  if (!this->manifest_.summary.empty())
    summary = this->manifest_.summary + "\n" + summary;
  if (summary.size() > SUMMARY_MAX)
    summary = summary.substr(0, SUMMARY_MAX - 3) + "...";
  const auto state = needed ? update::UPDATE_STATE_AVAILABLE : update::UPDATE_STATE_NO_UPDATE;
  const std::string installed = this->installed_text_();
  if (!this->have_report_)
    this->set_report_text_(port_report_text(planned));   // before the first job: the plan
  if (state == this->state_ && summary == this->update_info_.summary && installed == this->update_info_.current_version)
    return;
  this->state_ = state;
  this->update_info_.summary = summary;
  this->update_info_.current_version = installed;
  this->update_info_.has_progress = false;
  if (this->plan_.blocked) {
    this->set_status_("blocked: " + port_blocked_text(this->plan_, &this->inv_));
  } else {
    this->set_status_(needed ? "update available: " + this->manifest_.version : "up to date");
  }
  this->publish_state();
}

// ------------------------------------------------------------------------------------------------ install
void UltraFirmwareUpdate::perform(bool force) {
  if (force)   // (update.perform's force_update: the port update has none, a board on the release is current)
    ESP_LOGW(TAG, "'%s': force_update does nothing here: boards already on the release are current",
             this->get_name().c_str());
  if (this->run_ != nullptr) {
    ESP_LOGW(TAG, "'%s': a port job runs already", this->get_name().c_str());
    return;
  }
  if (this->hub_->port_off()) {
    ESP_LOGW(TAG, "'%s': its Diff-Power port's power mode is Off: no install", this->get_name().c_str());
    this->set_status_("not installed: its Diff-Power port's power mode is Off");
    return;
  }
  if (!this->have_release_) {
    ESP_LOGW(TAG, "'%s': no release known yet (check first)", this->get_name().c_str());
    this->set_status_("not installed: no release known yet (the last check failed?)");
    return;
  }
  this->replan_();
  PortReport report = port_report_from_plan(this->plan_, &this->inv_);
  if (this->plan_.blocked) {
    port_report_close(report, PJO_BLOCKED);
  } else if (report.count(PRR_PENDING) == 0) {
    port_report_close(report, PJO_DONE);   // nothing to send: every node current, skipped or unreachable
  } else {
    uint8_t refusal = PRF_NONE;
    int16_t rc = 0;
    if (!this->persist_idents_()) {
      ESP_LOGW(TAG, "'%s': the receivers' identities could not be written to the flash: no install",
               this->get_name().c_str());
      refusal = PRF_LEDGER;
    } else if (this->start_job_(this->plan_, &refusal, &rc)) {
      return;   // its end stores the report (end_job_)
    }
    report.refusal = refusal;
    report.job_rc = rc;
    port_report_close(report, PJO_REFUSED);
  }
  this->finish_job_(report);
  this->replan_();
}

// ------------------------------------------------------------------------------------------------ recover
// One silent known receiver's Recover firmware button: its release image, sent blind from its last trusted identity
// (the ledger), as the sender's ultra_update_job_recover (a board left revoked in its recovery bootloader answers no
// ordinary poll). "Update all" refuses while a known board is silent; this is the explicit action for one. It ends
// verified when the board is found again with the image, else "recovery sent".
void UltraFirmwareUpdate::recover(uint16_t board) {
  if (this->run_ != nullptr) {
    ESP_LOGW(TAG, "'%s': a port job runs already: B%u not recovered", this->get_name().c_str(), board + 1);
    return;
  }
  if (this->hub_->port_off()) {
    this->set_status_("not recovered: its Diff-Power port's power mode is Off");
    return;
  }
  if (!this->have_release_) {
    this->set_status_("not recovered: no release known yet (the last check failed?)");
    return;
  }
  this->replan_();   // (a fresh ledger)
  const PortPlan plan = build_recover_plan(this->inv_, this->manifest_, this->hub_->port_type(), board);
  PortReport report = port_report_from_plan(plan, &this->inv_);
  if (plan.blocked) {
    port_report_close(report, PJO_BLOCKED);
  } else {
    uint8_t refusal = PRF_NONE;
    int16_t rc = 0;
    if (!this->persist_idents_()) {
      refusal = PRF_LEDGER;
    } else if (this->start_job_(plan, &refusal, &rc)) {
      ESP_LOGI(TAG, "'%s': recovering B%u: its image %s sent blind", this->get_name().c_str(), board + 1,
               this->manifest_.version.c_str());
      return;
    }
    report.refusal = refusal;
    report.job_rc = rc;
    port_report_close(report, PJO_REFUSED);
  }
  this->finish_job_(report);
  this->replan_();
}

/// One port job: the plan it runs, the sender job's inputs and state, its transport (the task's side)
struct UltraFirmwareUpdate::JobRun final : public PortJobDriver::Transport {
  UltraHub *hub{nullptr};
  PortInventory inv;
  PortPlan plan;
  std::vector<std::unique_ptr<PortArtifactSource>> sources;
  std::vector<ultra_update_artifact_t> artifacts;
  std::vector<ultra_update_node_t> nodes;
  std::vector<ultra_update_result_t> results;
  std::vector<ultra_sender_board_t> boards;
  ultra_sender_t line{};
  ultra_update_job_options_t options{};
  ultra_update_job_t job{};
  std::vector<uint8_t> buffer;
  PortJobDriver driver;
  PortLease handover;
  std::atomic<bool> ended{false};
  bool aborted{false}, verifying{false};
  uint32_t verify_ms{0};
  TaskHandle_t task{nullptr};

  int64_t now_us() override { return esp_timer_get_time(); }
  int64_t frame_start_us() override { return this->hub->job_frame_start_us(); }
  bool line_free() override { return this->hub->job_line_free(); }
  bool send(const uint8_t *f, size_t n, const ultra_update_transport_t &t) override { return this->hub->job_send(f, n, t); }
  /// asked, never waited for: the main loop applies it (serve_lease): the hub's own frames stop, or it gets the line
  /// back. Taking it: true once the hub handed it over (UltraHub::lease_line: its response window read, the line
  /// driven again)
  bool lease(bool on) override { return this->handover.ask(on); }
  /// main loop: the task's ask applied to the hub (PortLease::serve)
  int serve_lease() { return this->handover.serve([this](bool on) { return this->hub->lease_line(on); }); }
};

static void free_image_buffer(uint8_t *img, size_t size) {
  RAMAllocator<uint8_t> alloc;
  alloc.deallocate(img, size);
}

bool UltraFirmwareUpdate::start_job_(const PortPlan &plan, uint8_t *refusal, int16_t *job_rc) {
  if (active_ != nullptr && active_ != this) {
    *refusal = PRF_BUSY;
    return false;
  }
  auto run = std::make_unique<JobRun>();
  run->hub = this->hub_;
  run->inv = this->inv_;
  run->plan = plan;
  ultra_timing_t timing;
  if (!this->hub_->job_timing(&timing)) {
    *refusal = PRF_TIMING;
    return false;
  }
  const uint16_t n = static_cast<uint16_t>(run->inv.nodes.size());
  run->boards.resize(n);
  ultra_sender_config_t c;
  ultra_sender_config_init(&c, n, 0);
  c.light_capacity = 0;
  c.clear_unused = 0;
  c.timing = timing;
  c.seed = esp_random();
  if (ultra_sender_init(&run->line, &c, run->boards.data(), nullptr, 0) != ULTRA_OK ||
      (this->hub_->job_digital() && ultra_sender_digital(&run->line, 1) != ULTRA_OK)) {
    *refusal = PRF_LINE;
    return false;
  }
  // its images, each downloaded when its subtransfer starts; its nodes; its options
  run->sources.resize(run->plan.artifacts.size());
  run->artifacts.resize(run->plan.artifacts.size());
  for (size_t a = 0; a < run->plan.artifacts.size(); a++) {
    const FwImageInfo &img = run->plan.artifacts[a].image;
    run->sources[a] = std::make_unique<PortArtifactSource>();
    run->sources[a]->init(img, fw_resolve_url(this->manifest_url_, img.url), UltraFirmwareUpdate::start_download_,
                          free_image_buffer);
    port_job_artifact(run->artifacts[a], img.descriptor, run->sources[a].get(), PortArtifactSource::read,
                      PortArtifactSource::prepare, PortArtifactSource::release);
  }
  const int64_t now_us = esp_timer_get_time();
  const uint32_t now_ms = millis();
  run->nodes.resize(n);
  for (uint16_t k = 0; k < n; k++)
    port_job_node(run->nodes[k], run->inv.nodes[k], run->plan.nodes[k], now_us, now_ms);
  run->results.resize(n);
  uint8_t rate = 0;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (this->hub_->has_telemetry())
    rate = this->hub_->telemetry().rate();
#endif
  port_job_options(run->options, run->plan, static_cast<uint8_t>(esp_random() | 1u), rate);
  run->options.verify_timeout_ms = PORT_JOB_VERIFY_MS;
  run->buffer.resize(std::max<size_t>(UPD_FRAME_BYTES, this->hub_->job_frame_max()));
  // "update all", or the recovery of one silent board (the same inventory, artifacts and checks)
  const int rc = run->plan.recover >= 0
                     ? ultra_update_job_recover(&run->job, &run->line, run->nodes.data(), n,
                                                static_cast<uint16_t>(run->plan.recover), run->artifacts.data(),
                                                static_cast<uint16_t>(run->artifacts.size()), run->results.data(),
                                                &run->options, esp_timer_get_time())
                     : ultra_update_job_begin(&run->job, &run->line, run->nodes.data(), n, run->artifacts.data(),
                                              static_cast<uint16_t>(run->artifacts.size()), run->results.data(),
                                              &run->options, esp_timer_get_time());
  if (rc != 0) {
    *refusal = PRF_JOB;
    *job_rc = static_cast<int16_t>(rc);
    return false;
  }
  run->driver.begin(&run->job, run.get(), run->buffer.data(), static_cast<uint32_t>(run->buffer.size()));
  active_ = this;
  this->hub_->hold_power(true);
  if (xTaskCreate(UltraFirmwareUpdate::job_task_, "d2a_port_job", 4096, run.get(), 5, &run->task) != pdPASS) {
    active_ = nullptr;
    this->hub_->hold_power(false);
    *refusal = PRF_TASK;
    return false;
  }
  this->run_ = run.release();
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->update_info_.has_progress = true;
  this->update_info_.progress = 0.0f;
  this->progress_ms_ = 0;
  this->set_status_(plan.recover >= 0 ? "recovering B" + std::to_string(plan.recover)
                                       : "updating: " + std::to_string(plan.count(PA_SEND)) + " receivers");
  this->publish_state();
  return true;
}

// The transport task: the driver's steps, sleeping while nothing is due (the hub's frames wait in its encoder)
void UltraFirmwareUpdate::job_task_(void *arg) {
  auto *run = static_cast<JobRun *>(arg);
  for (;;) {
    const PortJobDriver::Step s = run->driver.step();
    if (s == PortJobDriver::DONE)
      break;
    if (s == PortJobDriver::SENT)
      continue;
    if (s == PortJobDriver::VERIFYING || s == PortJobDriver::LEASING || run->driver.due_us() > esp_timer_get_time())
      vTaskDelay(1);
  }
  run->ended.store(true);
  vTaskDelete(nullptr);
}

void UltraFirmwareUpdate::service_job_() {
  JobRun *run = this->run_;
  // the lease the task asks for, never waiting for the task (an open response window: asked again at the next pass);
  // handed back, the receivers' identity and FW_UPDATE are read again
  const int served = run->serve_lease();
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (served == 0 && this->hub_->has_telemetry())
    this->hub_->refresh_identities();
#else
  (void) served;
#endif
  const bool verifying = run->driver.with_job([](ultra_update_job_t *j) { return j->state == ULTRA_JOB_VERIFY; });
  if (verifying && !run->verifying)
    run->verify_ms = millis();
  run->verifying = verifying;
#ifdef USE_ULTRA_LINE_TELEMETRY
  if (verifying && this->hub_->has_telemetry())
    port_job_verify(run->driver, this->hub_->telemetry(), run->verify_ms, esp_timer_get_time(), millis());
#endif
  // its Diff-Power port set to Off meanwhile: the job stops (the port powers nothing for it)
  if (this->hub_->port_off() && !run->aborted)
    this->abort();
  if (millis() - this->progress_ms_ >= 1000) {
    this->progress_ms_ = millis();
    ultra_update_job_progress_t jp{};
    const int ok = run->driver.with_job(
        [&jp](ultra_update_job_t *j) { return ultra_update_job_progress(j, esp_timer_get_time(), &jp); });
    if (ok == 0) {
      const PortJobProgress p = port_job_progress(jp);
      this->update_info_.progress = 100.0f * p.fraction();
      const std::string what = run->plan.recover >= 0 ? (verifying ? "verifying the recovery of B" : "recovering B") +
                                                              std::to_string(run->plan.recover) + ": "
                                                        : verifying ? "verifying " : "updating ";
      this->set_status_(what + port_job_progress_text(p));
      this->publish_state();
    }
  }
  if (run->ended.load())
    this->end_job_();
}

void UltraFirmwareUpdate::end_job_() {
  JobRun *run = this->run_;
  PortReport report = port_report_from_plan(run->plan, &run->inv);
  port_job_take_results(report, run->results.data(), run->results.size());
  // a subtransfer the sender refused at its begin: its code, so the report names it (its nodes are REFUSED)
  report.job_rc = static_cast<int16_t>(
      run->driver.with_job([](ultra_update_job_t *j) { return j != nullptr ? j->begin_code : 0; }));
  port_report_close(report, run->aborted || run->driver.transport_failed() ? PJO_ABORTED : PJO_DONE);
  this->hub_->lease_line(false);   // (also a lease asked for and not handed over yet: the hub's own frames again)
  this->hub_->hold_power(false);
  delete run;
  this->run_ = nullptr;
  if (active_ == this)
    active_ = nullptr;
  this->update_info_.has_progress = false;
  this->finish_job_(report);
  this->state_ = update::UPDATE_STATE_UNKNOWN;   // (replan_ publishes the new one)
  this->replan_();
}

// An image's download: its own task (the HTTP client blocks), the result to its source
bool UltraFirmwareUpdate::start_download_(PortArtifactSource *src) {
  return active_ != nullptr &&
         xTaskCreate(UltraFirmwareUpdate::download_task_, "d2a_fw_dl", 8192, src, 1, nullptr) == pdPASS;
}

void UltraFirmwareUpdate::download_task_(void *arg) {
  auto *src = static_cast<PortArtifactSource *>(arg);
  UltraFirmwareUpdate *self = active_;
  const size_t size = src->image().size;
  RAMAllocator<uint8_t> alloc;  // PSRAM when there is some
  uint8_t *img = size > UPD_DESC_BYTES && size <= UPD_DESC_BYTES + UPD_INDEX_REACH ? alloc.allocate(size) : nullptr;
  std::string err;
  if (self == nullptr || img == nullptr) {
    err = "no memory for the image (" + std::to_string(size / 1024 + 1) + " KB of heap)";
  } else {
    auto c = self->http_->get(src->url());
    if (c == nullptr || c->status_code != http_request::HTTP_STATUS_OK) {
      err = "cannot download the image (HTTP " + std::to_string(c ? c->status_code : -1) + ")";
    } else if (c->content_length != 0 && c->content_length != size) {
      err = "the server's image is not the size the manifest gives";
    } else {
      size_t got = 0;
      uint32_t last = millis();
      while (got < size && src->state() == PortArtifactSource::LOADING) {
        const int r = c->read(img + got, std::min<size_t>(1024, size - got));
        const auto res = http_request::http_read_loop_result(r, last, self->http_->get_timeout(), c->is_read_complete());
        if (res == http_request::HttpReadLoopResult::RETRY)
          continue;
        if (res != http_request::HttpReadLoopResult::DATA)
          break;
        got += static_cast<size_t>(r);
      }
      if (got != size)
        err = "the download stopped at " + std::to_string(got) + " of " + std::to_string(size) + " bytes";
    }
    if (c != nullptr)
      c->end();
  }
  uint8_t sha[32];
  if (err.empty() && !fw_sha256(img, size, sha))
    err = "SHA-256 failed";
  if (err.empty()) {
    src->loaded(img, size, sha);   // (checked there; freed there when the job let go meanwhile)
  } else {
    if (img != nullptr)
      alloc.deallocate(img, size);
    src->failed(err.c_str());
  }
  vTaskDelete(nullptr);
}

void UltraFirmwareUpdate::abort() {
  if (this->run_ == nullptr) {
    ESP_LOGI(TAG, "'%s': no port job runs", this->get_name().c_str());
    return;
  }
  this->run_->aborted = true;
  this->run_->driver.with_job([](ultra_update_job_t *j) {
    ultra_update_job_abort(j);
    return 0;
  });
  this->set_status_("aborting: boards that had started erasing wait in their bootloader for the next job");
}

void UltraFirmwareUpdate::on_shutdown() {
  if (this->run_ == nullptr)
    return;
  ESP_LOGW(TAG, "'%s': shutdown during a port job: aborting it", this->get_name().c_str());
  this->abort();
  // At most 30 s for the ABORT frames, not the job's full quiet (after an ANNOUNCE without DATA the line is held for
  // its transit hold, up to 114.34 s, + 2,100 ms): nothing lasting depends on it.
  // After the reboot the line is LOW for > 2 s, so every board re-acquires (3 valid trailers before any command) and
  // the hub's first frames are its DARK preamble: a transit app still in its hold passes one whole DARK frame, whose
  // decoded trailer ends the hold (an undecodable one: the hold ends on its own clock), so no forward stays stuck. A
  // target bootloader before its revoke times out and boots its intact app; after it, it stays dark in recovery
  // (outputs safe) however long we wait, as after a power cut mid-erase: its Recover firmware button sends it its image
  // (its last trusted identity, PortIdentityStore).
  static constexpr uint32_t SHUTDOWN_ABORT_MS = 30000;
  const uint32_t t0 = millis();
  while (!this->run_->ended.load() && millis() - t0 < SHUTDOWN_ABORT_MS) {
    this->run_->serve_lease();
    delay(1);
    App.feed_wdt();
  }
}

void UltraFirmwareUpdate::finish_job_(const PortReport &report) {
  this->report_ = report;
  this->have_report_ = true;
  auto store = std::make_unique<PortReportStore>();
  port_report_store(report, *store);
  this->report_pref_.save(store.get());
  this->log_report_();
  const std::string text = port_report_text(report);
  this->set_report_text_(text);
  this->set_status_(text.substr(0, text.find('\n')));
}

// The whole report in the log, a line each (never cut: every board with its result and reason; the report sensor in
// Home Assistant keeps 255 characters and points here for the rest)
void UltraFirmwareUpdate::log_report_() {
  if (!this->have_report_)
    return;
  const std::vector<PortReportLine> lines = port_report_lines(this->report_);
  ESP_LOGI(TAG, "'%s': last port update: %s", this->get_name().c_str(), lines[0].text.c_str());
  for (size_t k = 1; k < lines.size(); k++)
    ESP_LOGI(TAG, "'%s':   %s", this->get_name().c_str(), lines[k].text.c_str());
}

void UltraFirmwareUpdate::loop() {
  if (this->run_ != nullptr) {
    this->service_job_();
    return;
  }
  if (this->have_release_ && millis() - this->plan_ms_ >= PLAN_MS)
    this->replan_();   // (the receivers' identities come in after the start, a board may be swapped)
  else if (!this->have_release_ && millis() - this->remember_ms_ >= PLAN_MS)
    this->remember_(this->inventory_(millis()));   // (no release yet: the ledger still learns its receivers)
}

// ------------------------------------------------------------------------------------------------ channel select
#ifdef USE_SELECT
void FwChannelSelect::setup() {
  this->pref_ = this->make_entity_preference<uint32_t>();
  uint32_t index;
  uint8_t channel = this->update_->get_channel();  // the YAML's channel
  if (this->pref_.load(&index) && index < 3)
    channel = static_cast<uint8_t>(1u << index);
  this->update_->change_channel(channel, false);
  this->publish_state(channel == FW_CH_ALPHA ? 2 : channel == FW_CH_BETA ? 1 : 0);
}

void FwChannelSelect::control(size_t index) {
  if (index >= 3)
    return;
  uint32_t stored = static_cast<uint32_t>(index);
  this->pref_.save(&stored);
  this->publish_state(index);
  this->update_->change_channel(static_cast<uint8_t>(1u << index), true);
}
#endif

}  // namespace esphome::ultra_line

#endif
