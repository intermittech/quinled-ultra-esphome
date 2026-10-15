#include "esphome/core/defines.h"
#include "fw_update.h"

#if defined(USE_ESP32) && defined(USE_DIG2ANALOG_ULTRA_UPDATE)

#include "esphome/components/json/json_util.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>

namespace esphome::dig2analog_ultra {

static const char *const TAG = "dig2analog_ultra.update";

static constexpr uint32_t INITIAL_CHECK_ID = 0x44324146;  // set_interval id: the first check once the network is up
static constexpr uint32_t INITIAL_CHECK_MS = 10000;
static constexpr uint32_t PUBLISH_MS = 1000;            // progress / status at most this often
static constexpr size_t MANIFEST_MAX = 16384;
static constexpr size_t SIG_MAX = 160;                   // DER ECDSA P-256: 70-72 bytes
static constexpr uint32_t POWER_SETTLE_MS = 200;         // after the supply is ready: the boards' apps run
static constexpr uint32_t POWER_TIMEOUT_MS = 30000;
static constexpr float DOWNLOAD_SHARE = 15.0f;           // progress: download 0-15 %, session 15-100 %

static const char *const BLINK_LEGEND =
    "Blinks: 3 fast + N = updated (N = minor + 1), 1 short = had it, 1 long = rejected (on the status LED from "
    "0.6.9), slow 1 Hz = waiting in recovery";

UltraFirmwareUpdate *UltraFirmwareUpdate::active_ = nullptr;
uint8_t UltraFirmwareUpdate::entities_ = 0;

// The image shared by the lines (fw_image_cache.h): with several update entities it stays this long after its last
// session, so the lines of an "update all" (queued one after the other) download it once.
static constexpr uint32_t IMAGE_HOLD_MS = 60000;

static void free_image_buffer(uint8_t *img, size_t size) {
  RAMAllocator<uint8_t> alloc;
  alloc.deallocate(img, size);
}
static FwImageCache image_cache(free_image_buffer);

// Inputs copied on the main loop (the tasks never read members that the main loop may change), outputs for defer().
struct UltraFirmwareUpdate::CheckResult {
  UltraFirmwareUpdate *self{nullptr};
  uint8_t channel{0};
  std::string url;
  FwManifest manifest;
  FwImageInfo image;
  std::string image_url;
  std::string error;
  const char *key{nullptr};
};

struct UltraFirmwareUpdate::DownloadResult {
  UltraFirmwareUpdate *self{nullptr};
  FwImageInfo image;
  std::string url;
  uint32_t version{0};
  uint8_t hw_rev{0};
  uint8_t *img{nullptr};
  std::string error;
};

// ------------------------------------------------------------------------------------------------ setup / config
void UltraFirmwareUpdate::setup() {
  entities_++;
  this->pref_ = this->make_entity_preference<uint32_t>();
  if (!this->pref_.load(&this->installed_))
    this->installed_ = 0;
  this->update_info_.title = this->hw_id_ == UPD_HW_ID_DP_ULTRA_RX ? "QuinLED Diff-Power Ultra receivers"
                                                                    : "QuinLED Dig2Analog-Ultra-5ch";
  this->update_info_.current_version = this->installed_text_();
  this->publish_state();
  // without signing keys (public_keys: production before QuinLED provisions them) nothing can install: say so
  this->set_status_(this->verify_ && this->nkeys_ == 0
                        ? "no signing keys in this build: no release installs yet (QuinLED provisions the production "
                          "keys with the firmware's first release)"
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
  ESP_LOGCONFIG(TAG,
                "QuinLED dig2analog board firmware '%s':\n"
                "  Channel: %s, manifest %s\n"
                "  Signature: %s (%u public keys)\n"
                "  Boards: hardware %u revision %u, %u passes\n"
                "  Auto retry: %u (%s)\n"
                "  Installed on this line: %s",
                this->get_name().c_str(), fw_channel_name(this->channel_),
                fw_manifest_url(this->source_, this->channel_).c_str(), this->verify_ ? "checked" : "NOT checked",
                static_cast<unsigned>(this->nkeys_), this->hw_id_, this->hw_rev_, this->passes_, this->auto_retry_,
                this->line_answers_() ? "while a receiver isn't on the release" : "blind: a one-way line",
                this->installed_text_().c_str());
  if (this->verify_ && this->nkeys_ == 0)
    ESP_LOGW(TAG, "  No signing keys in this build: no release installs (QuinLED provisions the production keys with the "
                  "firmware's first release; bench builds: public_keys: test with your own source)");
}

std::string UltraFirmwareUpdate::installed_text_() const {
  return this->installed_ ? format_fw_version(this->installed_) : std::string("unknown");
}

void UltraFirmwareUpdate::set_status_(const std::string &status) {
  // (Home Assistant takes up to 255 characters)
  const std::string text = status.size() > 255 ? status.substr(0, 252) + "..." : status;
  if (text == this->status_text_)
    return;
  this->status_text_ = text;
  ESP_LOGD(TAG, "'%s': %s", this->get_name().c_str(), text.c_str());
#ifdef USE_TEXT_SENSOR
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(text);
#endif
}

void UltraFirmwareUpdate::publish_progress_(float percent, bool force_publish) {
  this->update_info_.has_progress = true;
  this->update_info_.progress = percent;
  const uint32_t now = millis();
  if (force_publish || now - this->last_publish_ms_ >= PUBLISH_MS) {
    this->last_publish_ms_ = now;
    this->publish_state();
  }
}

void UltraFirmwareUpdate::change_channel(uint8_t channel, bool check) {
  if (fw_channel_name(channel) == nullptr || channel == this->channel_)
    return;
  this->channel_ = channel;
  this->have_release_ = false;
  this->latest_ = 0;
  this->update_info_.latest_version.clear();
  if (!check)
    return;
  ESP_LOGI(TAG, "'%s': channel %s", this->get_name().c_str(), fw_channel_name(channel));
  if (this->phase_ == IDLE) {
    this->state_ = update::UPDATE_STATE_UNKNOWN;
    this->publish_state();
  }
  this->update();
}

// ------------------------------------------------------------------------------------------------ check
void UltraFirmwareUpdate::update() {
  if (this->check_task_handle_ != nullptr)
    return;  // one already runs
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
      m.name = root["name"] | "QuinLED Dig2Analog-Ultra-5ch";
      m.channel = root["channel"].as<std::string>();
      m.version = root["version"].as<std::string>();
      m.release_url = root["release_url"] | "";
      m.summary = root["summary"] | "";
      JsonArray images = root["images"].as<JsonArray>();
      for (JsonObject img : images) {
        FwImageInfo info;
        info.hw_id = img["hw_id"] | 0;
        info.hw_rev_mask = img["hw_rev_mask"] | 0;
        info.min_bootloader = img["min_bootloader"] | 0;
        info.size = img["size"] | 0;
        info.sha256 = img["sha256"] | "";
        info.url = img["url"] | "";
        m.images.push_back(info);
      }
      return true;
    });
    const char *err = parsed ? fw_check_manifest(m, r->channel) : "the manifest is not valid JSON";
    if (err != nullptr) {
      r->error = err;
    } else {
      const int i = fw_pick_image(m, self->hw_id_, self->hw_rev_);
      if (i < 0) {
        r->error = "the release has no image for board revision " + std::to_string(self->hw_rev_);
      } else {
        r->image = m.images[i];
        r->image_url = fw_resolve_url(r->url, r->image.url);
      }
    }
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
    if (this->phase_ == IDLE)
      this->set_status_("check failed: " + r->error);
    return;
  }
  this->status_clear_warning();
  ESP_LOGI(TAG, "'%s': %s %s (signed by the %s), installed here: %s", this->get_name().c_str(),
           fw_channel_name(r->channel), r->manifest.version.c_str(), r->key ? r->key : "no key (unchecked)",
           this->installed_text_().c_str());
  this->have_release_ = true;
  this->latest_ = r->manifest.version_word;
  this->image_ = r->image;
  this->image_url_ = r->image_url;
  this->update_info_.title = r->manifest.name;
  this->update_info_.latest_version = r->manifest.version;
  this->update_info_.summary = r->manifest.summary;
  this->update_info_.release_url = r->manifest.release_url;
  this->update_info_.firmware_url = r->image_url;
  this->update_info_.current_version = this->installed_text_();
  if (this->phase_ != IDLE)
    return;  // an install runs: its end publishes
  const bool available = fw_update_available(this->installed_, this->latest_);
  const bool newly = available && this->state_ != update::UPDATE_STATE_AVAILABLE;
  this->state_ = available ? update::UPDATE_STATE_AVAILABLE : update::UPDATE_STATE_NO_UPDATE;
  // an image for a newer bootloader says so: boards with an older one reject it (1 long blink)
  const std::string needs =
      r->image.min_bootloader > 1 ? " (needs bootloader " + std::to_string(r->image.min_bootloader) + ")" : "";
  this->set_status_(available ? "update available: " + r->manifest.version + needs : "up to date");
  this->publish_state();
  if (newly)
    this->get_update_available_trigger()->trigger(this->update_info_);
}

// ------------------------------------------------------------------------------------------------ install
void UltraFirmwareUpdate::perform(bool force) {
  if (this->phase_ != IDLE) {
    ESP_LOGW(TAG, "'%s': an install already runs", this->get_name().c_str());
    return;
  }
  if (this->hub_->port_off()) {
    ESP_LOGW(TAG, "'%s': its Diff-Power port's power mode is Off: no install", this->get_name().c_str());
    this->set_status_("failed: its Diff-Power port's power mode is Off");
    return;
  }
  if (!this->have_release_) {
    ESP_LOGW(TAG, "'%s': no release known yet (check first)", this->get_name().c_str());
    this->set_status_("failed: no release known yet (the last check failed?)");
    return;
  }
  if (!force && !fw_update_available(this->installed_, this->latest_)) {
    ESP_LOGI(TAG, "'%s': this line already has %s (force_update sends it again)", this->get_name().c_str(),
             this->update_info_.latest_version.c_str());
    return;
  }
  this->force_ = force;
  this->recover_ = false;
  this->power_cycle_ = false;
  this->lead_in_ms_ = 500;
  this->retry_.begin(force ? 0 : this->auto_retry_, this->line_answers_());   // never with FORCE
  this->start_install_();
}

// Every board of the line answers (Diff-Power receivers with telemetry): the retries check their versions.
bool UltraFirmwareUpdate::line_answers_() const {
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
  return this->hub_->has_telemetry() && this->hub_->telemetry().receivers() >= this->hub_->get_num_boards();
#else
  return false;
#endif
}

void UltraFirmwareUpdate::recover(uint32_t lead_in_ms, bool power_cycle) {
  if (this->phase_ != IDLE || !this->have_release_) {
    ESP_LOGW(TAG, "'%s': recover needs a known release and no install running", this->get_name().c_str());
    return;
  }
  if (this->hub_->port_off()) {
    ESP_LOGW(TAG, "'%s': its Diff-Power port's power mode is Off: no recovery", this->get_name().c_str());
    this->set_status_("failed: its Diff-Power port's power mode is Off");
    return;
  }
  this->force_ = true;
  this->recover_ = true;
  this->retry_.begin(0, false);   // (forced: no retries)
  this->power_cycle_ = power_cycle && this->hub_->has_power_supply();
  this->lead_in_ms_ = std::max<uint32_t>(lead_in_ms, 500);
  // the boards come back power_cycle_ms() after the cycle starts (the supply's keep_on_time, 1.5 s off): the ANNOUNCE
  // lead-in must still run then, with time for their boot
  if (this->power_cycle_)
    this->lead_in_ms_ = std::max(this->lead_in_ms_, this->hub_->power_cycle_ms() + 3000);
  this->start_install_();
}

void UltraFirmwareUpdate::start_install_() {
  // this release, also when the install waits for another line meanwhile (a check or channel change may come)
  this->job_image_ = this->image_;
  this->job_url_ = this->image_url_;
  this->job_version_ = this->latest_;
  this->dl_abort_.store(false);
  this->dl_bytes_.store(0);
  this->off_stop_ = false;
  this->phase_ = QUEUED;
  this->phase_ms_ = millis();
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->set_status_(active_ != nullptr && active_ != this ? "queued: another line is updating" : "downloading");
  this->publish_progress_(0.0f, true);
}

void UltraFirmwareUpdate::download_task_(void *arg) {
  auto *r = static_cast<DownloadResult *>(arg);
  UltraFirmwareUpdate *self = r->self;
  const size_t size = r->image.size;
  RAMAllocator<uint8_t> alloc;  // PSRAM when there is some
  r->img = size > UPD_DESC_BYTES && size <= UPD_DESC_BYTES + UPD_APP_BODY_MAX ? alloc.allocate(size) : nullptr;
  if (r->img == nullptr) {
    r->error = "no memory for the image (" + std::to_string(size / 1024 + 1) + " KB of heap)";
  } else {
    auto c = self->http_->get(r->url);
    if (c == nullptr || c->status_code != http_request::HTTP_STATUS_OK) {
      r->error = "cannot download the image (HTTP " + std::to_string(c ? c->status_code : -1) + ")";
    } else if (c->content_length != 0 && c->content_length != size) {
      r->error = "the server's image is not the size the manifest gives";
    } else {
      size_t n = 0;
      uint32_t last = millis();
      while (n < size) {
        if (self->dl_abort_.load()) {
          r->error = "aborted";
          break;
        }
        const int got = c->read(r->img + n, std::min<size_t>(1024, size - n));
        const auto res = http_request::http_read_loop_result(got, last, self->http_->get_timeout(), c->is_read_complete());
        if (res == http_request::HttpReadLoopResult::RETRY)
          continue;
        if (res != http_request::HttpReadLoopResult::DATA)
          break;
        n += static_cast<size_t>(got);
        self->dl_bytes_.store(n);
      }
      if (r->error.empty() && n != size)
        r->error = "the download stopped at " + std::to_string(n) + " of " + std::to_string(size) + " bytes";
    }
    if (c != nullptr)
      c->end();
    c.reset();
    if (r->error.empty()) {
      uint8_t sha[32];
      const char *err = fw_sha256(r->img, size, sha) ? fw_check_image(r->img, size, sha, r->image, r->version, r->hw_rev)
                                                     : "SHA-256 failed";
      if (err != nullptr)
        r->error = err;
    }
    if (!r->error.empty()) {
      alloc.deallocate(r->img, size);
      r->img = nullptr;
    }
  }
  self->defer([self, r]() {
    self->on_downloaded_(r);
    delete r;
  });
  vTaskDelete(nullptr);
}

void UltraFirmwareUpdate::on_downloaded_(DownloadResult *r) {
  if (!r->error.empty()) {
    this->finish_(r->error == "aborted" ? r->error : "failed: " + r->error, false);
    return;
  }
  this->img_ = r->img;
  this->img_size_ = r->image.size;
  this->job_version_ = r->version;
  if (!image_cache.put(r->image.sha256, r->img, r->image.size))
    ESP_LOGD(TAG, "Another image is in use: this one is not shared");
  if (this->dl_abort_.load()) {
    this->finish_("aborted", false);  // came in as the download finished
    return;
  }
  this->image_ready_();
}

// A downloaded or shared image that passed the checks: the supply, then the session
void UltraFirmwareUpdate::image_ready_() {
  this->phase_ = WAIT_POWER;
  this->phase_ms_ = millis();
  this->hub_->hold_power(true);
  this->publish_progress_(DOWNLOAD_SHARE, true);
  if (this->recover_) {
    this->start_session_();  // the lead-in streams while the boards power up
  } else {
    this->set_status_("waiting for the boards' power");
  }
}

void UltraFirmwareUpdate::start_session_() {
  UpdateSession::Options o;
  o.lead_in_us = this->lead_in_ms_ * 1000;
  o.passes = this->passes_;
  o.force = this->force_;
  if (!this->hub_->start_update(this->img_, this->img_size_, o)) {
    this->finish_("failed: the data line is busy", false);
    return;
  }
  this->phase_ = SENDING;
  this->phase_ms_ = millis();
  if (this->recover_ && this->power_cycle_) {
    this->hub_->power_cycle();
    this->set_status_("recovery: power-cycling the boards (off after the supply's keep-on time, " +
                      std::to_string((this->hub_->power_cycle_ms() + 999) / 1000) + " s), ANNOUNCE for " +
                      std::to_string(this->lead_in_ms_ / 1000) + " s");
  } else if (this->recover_) {
    this->set_status_("recovery: power-cycle the boards on this line now (ANNOUNCE for " +
                      std::to_string(this->lead_in_ms_ / 1000) + " s)");
  } else {
    this->set_status_(this->retry_.used() != 0 ? this->retry_.retry_text(this->job_version_) : "sending");
  }
}

// A session's result is in (its PING burst is out, the new firmware confirmed): the same session again (blind), the
// receivers' check (telemetry), or the end. The image and the supply stay held until the end.
void UltraFirmwareUpdate::session_done_() {
  const uint32_t now = millis();
  switch (this->retry_.session_done(now)) {
    case AutoRetry::RETRY:
      ESP_LOGI(TAG, "'%s': %s", this->get_name().c_str(), this->retry_.retry_text(this->job_version_).c_str());
      this->start_session_();
      break;
    case AutoRetry::CHECK:
      this->phase_ = CHECKING;
      this->phase_ms_ = now;
      this->set_status_(this->retry_.checking_text(this->hub_->get_num_boards()));
      break;
    default: {
      std::string s = "done: " + format_fw_version(this->job_version_) + " sent to the " +
                      std::to_string(this->hub_->get_num_boards()) + " boards of this line";
      if (this->retry_.used() != 0)
        s += " in " + std::to_string(this->retry_.used() + 1) + " sessions";
      this->finish_(s + ". " + BLINK_LEGEND, true);
      break;
    }
  }
}

void UltraFirmwareUpdate::free_image_() {
  if (this->img_ == nullptr)
    return;
  if (image_cache.holds(this->img_)) {
    image_cache.release(millis());   // freed by the cache when no other line wants it
  } else {
    free_image_buffer(this->img_, this->img_size_);
  }
  this->img_ = nullptr;
}

// QUEUED, our turn: the image another line already downloaded and checked (same SHA-256), or a download
bool UltraFirmwareUpdate::take_shared_image_() {
  uint8_t *img = image_cache.acquire(this->job_image_.sha256, this->job_image_.size);
  if (img == nullptr)
    return false;
  uint8_t sha[32];
  const char *err = fw_hex_to_bytes(this->job_image_.sha256, sha, sizeof(sha))
                        ? fw_check_image(img, this->job_image_.size, sha, this->job_image_, this->job_version_,
                                         this->hw_rev_)
                        : "bad SHA-256 in the manifest";
  this->img_ = img;
  this->img_size_ = this->job_image_.size;
  if (err != nullptr) {   // e.g. this line's board revision is not in the image
    this->finish_(std::string("failed: ") + err, false);
    return true;
  }
  ESP_LOGI(TAG, "'%s': using the image another line downloaded", this->get_name().c_str());
  this->publish_progress_(DOWNLOAD_SHARE, true);
  this->image_ready_();
  return true;
}

void UltraFirmwareUpdate::finish_(const std::string &status, bool installed) {
  this->free_image_();
  if (active_ == this)
    active_ = nullptr;
  this->hub_->hold_power(false);
  this->phase_ = IDLE;
  if (installed) {
    this->installed_ = this->job_version_;
    this->pref_.save(&this->installed_);
    this->update_info_.current_version = this->installed_text_();
  }
  this->update_info_.has_progress = false;
  this->state_ = !this->have_release_ ? update::UPDATE_STATE_UNKNOWN
                 : fw_update_available(this->installed_, this->latest_) ? update::UPDATE_STATE_AVAILABLE
                                                                        : update::UPDATE_STATE_NO_UPDATE;
  ESP_LOGI(TAG, "'%s': %s", this->get_name().c_str(), status.c_str());
  this->set_status_(status);
  this->publish_state();
}

void UltraFirmwareUpdate::abort() {
  switch (this->phase_) {
    case QUEUED:
    case WAIT_POWER:
      this->finish_("aborted", false);
      break;
    case DOWNLOADING:
      this->dl_abort_.store(true);  // the task ends with "aborted"
      break;
    case SENDING:
      this->hub_->abort_update();   // its result ends the install
      break;
    case CHECKING:
      this->finish_("aborted: no retry (boards that installed it keep it)", false);
      break;
    default:
      break;
  }
}

void UltraFirmwareUpdate::loop() {
  const uint32_t now = millis();
  image_cache.service(now, entities_ > 1 ? IMAGE_HOLD_MS : 0);
  // its Diff-Power port set to Off meanwhile: the install stops (the port powers nothing for it)
  if (this->phase_ != IDLE && !this->off_stop_ && this->hub_->port_off()) {
    this->off_stop_ = true;
    ESP_LOGW(TAG, "'%s': its Diff-Power port's power mode is now Off: the install stops", this->get_name().c_str());
    if (this->phase_ == DOWNLOADING || this->phase_ == SENDING) {
      this->abort();   // (it ends once its task stops)
    } else {
      this->finish_("aborted: its Diff-Power port's power mode is Off", false);
      return;
    }
  }
  switch (this->phase_) {
    case QUEUED: {
      if (active_ != nullptr && active_ != this)
        break;
      active_ = this;
      if (this->take_shared_image_())
        break;
      image_cache.trim();
      auto *r = new DownloadResult();
      r->self = this;
      r->image = this->job_image_;
      r->url = this->job_url_;
      r->version = this->job_version_;
      r->hw_rev = this->hw_rev_;
      this->phase_ = DOWNLOADING;
      this->set_status_("downloading");
      TaskHandle_t task;
      if (xTaskCreate(UltraFirmwareUpdate::download_task_, "d2a_fw_dl", 8192, r, 1, &task) != pdPASS) {
        delete r;
        this->finish_("failed: cannot start the download task", false);
      }
      break;
    }
    case DOWNLOADING:
      if (now - this->last_publish_ms_ >= PUBLISH_MS && this->job_image_.size != 0) {
        const uint32_t n = this->dl_bytes_.load();
        const size_t size = this->job_image_.size;
        this->set_status_("downloading: " + std::to_string(n / 1024) + " of " + std::to_string(size / 1024) + " KB");
        this->publish_progress_(DOWNLOAD_SHARE * static_cast<float>(n) / static_cast<float>(size), false);
      }
      break;
    case WAIT_POWER:
      if (this->hub_->is_power_ready() && now - this->phase_ms_ >= POWER_SETTLE_MS) {
        this->start_session_();
      } else if (now - this->phase_ms_ > POWER_TIMEOUT_MS) {
        this->finish_("failed: the boards' power supply did not come on", false);
      }
      break;
    case SENDING: {
      const auto res = this->hub_->take_update_result();
      if (res == UltraHub::UPD_DONE) {
        this->session_done_();
      } else if (res == UltraHub::UPD_ABORTED) {
        this->finish_("aborted: boards that had started erasing wait in their bootloader (slow red blink) for the "
                      "next install",
                      false);
      } else if (res == UltraHub::UPD_FAILED) {
        this->finish_("failed: the data line's transmitter stopped", false);
      } else if (now - this->last_publish_ms_ >= PUBLISH_MS && this->hub_->update_running()) {
        const float p = this->hub_->update_progress();
        if (!this->recover_ || now - this->phase_ms_ > this->lead_in_ms_) {
          this->set_status_(this->retry_.prefix() + "sending: pass " + std::to_string(this->hub_->update_pass() + 1) +
                            " of " + std::to_string(this->passes_) + ", " +
                            std::to_string(static_cast<int>(p * 100.0f)) + " %");
        } else if (this->power_cycle_ && this->hub_->power_cycle_state() == UltraHub::CYCLE_FAILED) {
          this->set_status_("recovery: the supply stayed on - power-cycle the boards by hand now");
        }
        this->publish_progress_(DOWNLOAD_SHARE + (100.0f - DOWNLOAD_SHARE) * p, false);
      }
      break;
    }
    case CHECKING: {   // the receivers' versions after a session (telemetry): done, or the session again
#ifdef USE_DIG2ANALOG_ULTRA_TELEMETRY
      const UpdateTarget t{this->job_version_, this->job_image_.hw_id, this->job_image_.hw_rev_mask,
                           this->job_image_.min_bootloader};
      const auto step = this->retry_.check(now, this->hub_->telemetry(), t);
      if (step == AutoRetry::RETRY) {
        ESP_LOGI(TAG, "'%s': %s", this->get_name().c_str(), this->retry_.retry_text(this->job_version_).c_str());
        this->start_session_();
      } else if (step == AutoRetry::FINISH) {
        this->finish_("done: " + this->retry_.result_text(this->job_version_), this->retry_.installed());
      }
#else
      this->finish_("done", true);
#endif
      break;
    }
    default:
      break;
  }
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

}  // namespace esphome::dig2analog_ultra

#endif
