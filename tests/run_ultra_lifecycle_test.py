"""Build and run tests/ultra_lifecycle_test.cpp with the zig C/C++ compiler (pip install ziglang).

The test runs methods of the component that need ESPHome / ESP-IDF (the light's setup, the update entity's install and
downloads, the hub's loop and transmit) with stand-ins for those. This script first cuts their bodies unchanged from
components/ultra_line into tests/build/lifecycle/*.inc, so the test always runs the current code.
"""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
COMP = ROOT / "components" / "ultra_line"
BUILD = HERE / "build" / "lifecycle"
ZIG = [sys.executable, "-m", "ziglang"]

# <name>.inc: the definitions in a source file that start with these signatures
PIECES = {
    "is_lit.inc": ("ultra_light.h", ["bool is_lit("]),
    "light_methods.inc": (
        "ultra_light.cpp",
        ["static uint8_t scene_fade(", "static uint16_t mired10(", "void UltraLight::setup_state(",
         "void UltraLight::send_opt_(", "void UltraLight::transition_done_(", "void UltraLight::lit_(",
         "void UltraLight::send_pixel(",
         "void UltraLight::send_stored_level_("],
    ),
    "update_methods.inc": (
        "fw_update.cpp",
        ["bool UltraFirmwareUpdate::fetch_(", "bool UltraFirmwareUpdate::add_inventory(",
         "PortInventory UltraFirmwareUpdate::inventory_(", "void UltraFirmwareUpdate::remember_(",
         "bool UltraFirmwareUpdate::persist_idents_(",
         "std::string UltraFirmwareUpdate::installed_text_(",
         "void UltraFirmwareUpdate::replan_(", "void UltraFirmwareUpdate::perform(",
         "void UltraFirmwareUpdate::finish_job_(", "void UltraFirmwareUpdate::log_report_(",
         "void UltraFirmwareUpdate::recover("],
    ),
    "hub_loop.inc": ("ultra_line.cpp", ["void UltraHub::loop("]),
    "hub_lease.inc": ("ultra_line.cpp", ["bool UltraHub::lease_line("]),
    "hub_readiness.inc": ("ultra_line.cpp", ["bool UltraHub::receivers_ready_(", "void UltraHub::receiver_reply_("]),
    "hub_job.inc": ("ultra_line.cpp", ["bool UltraHub::job_send("]),
    "hub_wake.inc": ("ultra_line.cpp", ["void UltraHub::wake_(", "void UltraHub::loop_fast_("]),
    "hub_line.inc": ("ultra_line.cpp", ["bool UltraHub::line_free_(", "bool UltraHub::line_out_("]),
    "hub_admission.inc": ("ultra_line.cpp", ["const char *UltraHub::line_profile_refusal_("]),
    "hub_transmit.inc": ("ultra_line.cpp", ["void UltraHub::transmit_("]),
    "hub_power.inc": ("ultra_line.cpp", ["void UltraHub::service_power_("]),
    "hub_start.inc": (
        "ultra_line.cpp",
        ["int64_t UltraHub::lead_us(", "int64_t UltraHub::command_lead_us_(", "int64_t UltraHub::start_time_us("],
    ),
    "hub_rw.inc": (
        "ultra_line.cpp",
        ["bool UltraHub::rw_due_(", "bool UltraHub::rx_settling(", "bool UltraHub::rw_uart_turn_(",
         "void UltraHub::rw_drop_turn_(", "bool IRAM_ATTR UltraHub::rw_done_isr_(", "void UltraHub::finish_window_(",
         "void UltraHub::rw_publish_(", "void UltraHub::rw_uart_rate_("],
    ),
    # the Diff-Power port (components/diffpower) and the hub's side of it
    "diffpower_methods.inc": (
        "../diffpower/diffpower.cpp",
        ["void DiffPowerPort::setup(", "void DiffPowerPort::loop(", "void DiffPowerPort::apply_(",
         "void DiffPowerPort::resend_(",
         "void DiffPowerPort::save_stats_(", "void DiffPowerPort::publish_(", "void DiffPowerPort::publish_wake_(",
         "void DiffPowerPort::shutdown_all(",
         "void DiffPowerPort::wake(",
         "bool DiffPowerPort::has_light(", "uint8_t PowerModeSelect::mode_of_(", "size_t PowerModeSelect::index_of_(",
         "void PowerModeSelect::setup(", "void PowerModeSelect::control("],
    ),
    # the ESP32's lifetime ledger on the preferences: its stored copies (design review, round 9)
    "hub_ledger.inc": (
        "ultra_line.cpp",
        ["// The ESP32's one lifetime ledger (every line's", "void UltraHub::ledger_service_(",
         "void UltraHub::ledger_save_("],
    ),
    # ... and its own counts of a line's receivers: the port's power-on / cut, their lit time (design review, round 9)
    "hub_ledger_line.inc": ("ultra_line.cpp", ["void UltraHub::ledger_line_("]),
    # the commissioning's hub side (round 12): one line at a time, the test level as a layer, its record
    "hub_commission.inc": (
        "ultra_line.cpp",
        ["void UltraHub::commission(", "void UltraHub::commission_all(", "void UltraHub::service_commission_(",
         "void UltraHub::commission_level_(", "void UltraHub::CommissionLayer::layer_shown("],
    ),
    # the receivers' settings from Home Assistant (the action set_receiver_setting, the Settings text)
    "hub_rx.inc": (
        "ultra_line.cpp",
        ["void UltraHub::rx_settings_setup_(", "void UltraHub::rx_settings_lights_(", "void UltraHub::rx_settings_save_(",
         "const char *UltraHub::line_name_(", "int UltraHub::rx_setting_key_(",
         "bool UltraHub::set_receiver_setting(", "std::string UltraHub::receiver_settings_text(",
         "bool UltraHub::is_line(", "void UltraHub::receiver_setting_action("],
    ),
    "hub_port.inc": (
        "ultra_line.cpp",
        ["void UltraHub::attach_port_(", "void UltraHub::send_power_off_notice(", "void UltraHub::pump_notice_(",
         "void UltraHub::service_notice_(", "bool UltraHub::boards_powered_(", "bool UltraHub::has_power_supply("],
    ),
    "receiver_methods.inc": (
        "ultra_receiver.cpp",
        ["static Pace live_pace(", "static void publish_paced(", "static Pace receiver_pace(", "static Pace port_pace(",
         "void UltraReceiver::value_(", "void UltraReceiver::text_(", "void UltraReceiver::update_(",
         "void UltraTelemetryPort::value_(", "void UltraTelemetryPort::update_(",
         "void UltraTelemetryPort::publish_health_(",
         "void UltraLightTelemetry::store_energy_(",
         "void UltraLightTelemetry::update_("],
    ),
}
# classes: the definition, then its ;
CLASSES = {}  # Effect lifecycle is covered by run_analog_effects_test.py.


def definition(text: str, signature: str) -> str:
    """The definition that starts with signature, through its closing brace (braces in comments / strings would
    confuse it: the methods cut here have none)."""
    start = text.index(signature)
    i, depth = text.index("{", start), 0
    while True:
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        i += 1
        if depth == 0:
            return text[start:i]


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    for name, (src, signatures) in PIECES.items():
        text = (COMP / src).read_text(encoding="utf-8")
        (BUILD / name).write_text("\n\n".join(definition(text, s) for s in signatures) + "\n", encoding="utf-8")
    for name, (src, signatures) in CLASSES.items():
        text = (COMP / src).read_text(encoding="utf-8")
        (BUILD / name).write_text("".join(definition(text, s) + ";\n\n" for s in signatures), encoding="utf-8")
    # the receiver entities' constants (the pages each value comes from)
    rx = (COMP / "ultra_receiver.cpp").read_text(encoding="utf-8")
    first, last = "static constexpr uint32_t ALL_NEW", "PG_SYNC = 1u << TP_SYNC;\n"
    (BUILD / "receiver_consts.inc").write_text(rx[rx.index(first):rx.index(last) + len(last)], encoding="utf-8")
    exe = BUILD / ("ultra_lifecycle_test.exe" if sys.platform == "win32" else "ultra_lifecycle_test")
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wno-unused-variable", "-Wno-unused-function",
                 "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-I{BUILD}",
                 str(HERE / "ultra_lifecycle_test.cpp"), "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
