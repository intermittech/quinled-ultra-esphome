#pragma once

// "Update all receivers on this port" (`update: - platform: ultra_line`, update.py): one update entity per data
// line, i.e. per port. It reads the channel's signed manifest from GitHub (fw_manifest.h, fw_signature.h) and keeps the
// port's plan current: the topology ledger of every position (port_inventory.h: the receivers' own identity, the
// boards the YAML provisions), and which image each node gets, family by family (port_plan.h). Install is one port
// job of the sender library (port_job.h): its transport task puts the job's frames on the leased line
// (port_job_driver.h), each image is downloaded only for its own subtransfer (port_artifact.h), the receivers confirm
// from fresh identity and FW_UPDATE pages, and the per-node results stay in the report (port_report.h), stored on the
// ESP32. Design: the protocol's docs/update.md (Port job, Job results).

#include "esphome/core/defines.h"

#if defined(USE_ESP32) && defined(USE_ULTRA_LINE_UPDATE)

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/update/update_entity.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "ultra_line.h"
#include "fw_manifest.h"
#include "port_artifact.h"
#include "port_inventory.h"
#include "port_job.h"
#include "port_job_driver.h"
#include "port_plan.h"
#include "port_report.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::ultra_line {

class UltraFirmwareUpdate final : public update::UpdateEntity, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;  // look for a new release (never installs by itself)
  void dump_config() override;
  void on_shutdown() override;   // a running port job aborts (its ABORT frames before the restart)
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  /// Update all receivers on this port to the channel's release (boards on its version are current; force_update
  /// changes nothing: the port update has no force)
  void perform(bool force) override;
  void check() override { this->update(); }

  // Configuration (codegen)
  void set_hub(UltraHub *hub) { this->hub_ = hub; }
  void set_http(http_request::HttpRequestComponent *http) { this->http_ = http; }
  void set_source(const std::string &source) { this->source_ = source; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_keys(const FwPublicKey *keys, size_t n) {
    this->keys_ = keys;
    this->nkeys_ = n;
  }
  void set_verify_signature(bool verify) { this->verify_ = verify; }
  /// A provisioned position (the YAML inventory): its 32-byte protected boot info, optional UID, layout and blackout
  bool add_inventory(uint16_t board, const std::vector<uint8_t> &boot_info, const std::vector<uint8_t> &uid,
                     bool return_channel, bool has_layout, uint32_t metadata_base, uint32_t metadata_bytes,
                     uint32_t bootloader_bytes, bool has_blackout, uint32_t blackout_ms);
#ifdef USE_TEXT_SENSOR
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }
  void set_report_sensor(text_sensor::TextSensor *s) { this->report_sensor_ = s; }
#endif

  /// The channel select (stored there): switch, then look again (check = false at boot)
  void change_channel(uint8_t channel, bool check);
  uint8_t get_channel() const { return this->channel_; }
  /// Stop the port job: ABORT frames (boards that started erasing wait in their bootloader for the next job)
  void abort();
  /// Recover one silent known receiver (its Recover firmware button): its release image, blind, from its last trusted
  /// identity; "update all" refuses while a known board is silent
  void recover(uint16_t board);
  /// The last job's whole report (every board, its result and reason; Home Assistant's report sensor shows what fits
  /// in 255 characters): for a lambda, e.g. an API action's response
  std::string full_report() const { return this->have_report_ ? port_report_full_text(this->report_) : std::string(); }

 protected:
  struct CheckResult;
  struct JobRun;   // one port job: its inputs, the sender job and the transport task (fw_update.cpp)
  static void check_task_(void *arg);
  /// The plan's job on the line: true when its task runs; false: refused (refusal, job_rc say why)
  bool start_job_(const PortPlan &plan, uint8_t *refusal, int16_t *job_rc);
  /// Main loop, while a job runs: the lease its task asks for, verification, progress, its end
  void service_job_();
  /// The job ended: its results into the stored report, the line and supply let go
  void end_job_();
  static void job_task_(void *arg);
  static bool start_download_(PortArtifactSource *src);
  static void download_task_(void *arg);
  bool fetch_(const std::string &url, std::vector<uint8_t> &out, size_t max_len);
  void on_checked_(CheckResult *r);
  /// The port's ledger now: every position, from the receivers' identity and the provisioning
  PortInventory inventory_(uint32_t now_ms) const;
  /// The plan from a fresh ledger and the release; publishes the entity's state, summary and versions
  void replan_();
  /// A job ended: its report stored and shown
  void finish_job_(const PortReport &report);
  /// The last job's whole report in the log, a line each
  void log_report_();
  void set_status_(const std::string &text);
  void set_report_text_(const std::string &text);
  std::string installed_text_() const;

  UltraHub *hub_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};
  std::string source_;
  uint8_t channel_{FW_CH_STABLE};
  const FwPublicKey *keys_{nullptr};
  size_t nkeys_{0};
  bool verify_{true};
  std::vector<PortProvision> provisions_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *report_sensor_{nullptr};
#endif

  // the last good check (main loop only)
  bool have_release_{false};
  FwManifest manifest_;
  std::string manifest_url_;

  TaskHandle_t check_task_handle_{nullptr};

  PortInventory inv_;   // the plan's ledger
  PortPlan plan_;
  uint32_t plan_ms_{0};
  PortReport report_;   // the last job's (stored)
  bool have_report_{false};
  ESPPreferenceObject report_pref_;
  ESPPreferenceObject ident_pref_;   // the receivers' last live identities (PortIdentityStore)
  PortIdentityStore idents_{};
  uint32_t remember_ms_{0};
  void remember_(const PortInventory &inv);
  /// The ledger on the flash now, read back (before a job: its sends can revoke an app); false: not kept
  bool persist_idents_();
  std::string status_text_, report_text_;
  JobRun *run_{nullptr};
  uint32_t progress_ms_{0};
  /// one port job at a time on this ESP32 (each image takes the heap while it goes out)
  static UltraFirmwareUpdate *active_;
};

#ifdef USE_SELECT
/// The firmware channel of an update entity (stable / beta / alpha), stored on the ESP32
class FwChannelSelect final : public select::Select, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::DATA; }  // before the entity's first check
  void set_update(UltraFirmwareUpdate *u) { this->update_ = u; }

 protected:
  void control(size_t index) override;
  UltraFirmwareUpdate *update_{nullptr};
  ESPPreferenceObject pref_;
};
#endif

template<typename... Ts> class AbortUpdateAction final : public Action<Ts...>, public Parented<UltraFirmwareUpdate> {
 public:
  void play(const Ts &...x) override { this->parent_->abort(); }
};

}  // namespace esphome::ultra_line

#endif
