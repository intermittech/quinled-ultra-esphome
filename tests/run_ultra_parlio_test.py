"""Build and run tests/ultra_parlio_test.cpp with the zig C/C++ compiler (pip install ziglang): the PARLIO lanes
(ultra_parlio.h) and the telemetry coordinator for lines sharing a UART, with the hub's own loop / transmit / window /
start / update code cut unchanged from components/dig2analog_ultra/dig2analog_ultra.cpp (as run_ultra_lifecycle_test.py
does).

Before it, the feature check: dig2analog_ultra.cpp and .h preprocessed with every combination of telemetry, PARLIO and
updates; every member the .cpp uses must be declared in the .h for that combination (a build with PARLIO lanes and no
telemetry once named a callback that only exists with telemetry).

Then the negative controls: the same checks against copies of the sources with one thing broken (a fix undone, or a
property the tests guard); each must fail. And the render's cost per chunk (-O2, no sanitizers).

Usage: python tests/run_ultra_parlio_test.py [--no-negative] [--no-bench] [--only TEXT (the negative controls whose
description has TEXT)]
"""

from itertools import product
from pathlib import Path
import re
import subprocess
import sys

from run_ultra_lifecycle_test import definition

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
COMP = ROOT / "components" / "dig2analog_ultra"
BUILD = HERE / "build" / "parlio"
ZIG = [sys.executable, "-m", "ziglang"]
SOURCES = ("dig2analog_ultra.cpp", "dig2analog_ultra.h", "ultra_parlio.h", "../diffpower/diffpower.cpp")

# <name>.inc: the definitions in dig2analog_ultra.cpp that start with these signatures
PIECES = {
    "hub_loop.inc": ["void UltraHub::loop("],
    "hub_transmit.inc": ["void UltraHub::transmit_(", "esp_err_t UltraHub::lane_post_("],
    "hub_rw.inc": [
        "bool UltraHub::rw_due_(", "bool UltraHub::rx_settling(", "bool UltraHub::rw_uart_turn_(",
        "void UltraHub::rw_drop_turn_(",
        "bool IRAM_ATTR UltraHub::rw_done_isr_(", "void IRAM_ATTR UltraHub::rw_lane_release_(",
        "void UltraHub::finish_window_(", "void UltraHub::rw_publish_(",
    ],
    "hub_start.inc": ["int64_t UltraHub::lead_us(", "int64_t UltraHub::command_lead_us_(", "int64_t UltraHub::start_time_us("],
    "hub_update_start.inc": ["bool UltraHub::start_update("],
    "hub_update_lane.inc": ["static void upd_stamp(", "uint8_t UltraHub::update_lane_("],
    "hub_port.inc": [
        "void UltraHub::resync(", "bool UltraHub::boards_powered_(", "bool UltraHub::line_free_(",
        "bool UltraHub::line_out_(", "void UltraHub::attach_port_(", "void UltraHub::send_power_off_notice(",
        "void UltraHub::pump_notice_(", "void UltraHub::service_notice_(",
    ],
}
# port.inc: the Diff-Power port's own (components/diffpower/diffpower.cpp)
PORT_PIECES = ["void DiffPowerPort::loop(", "void DiffPowerPort::apply_(", "void DiffPowerPort::shutdown_all("]

# (what breaks, file, the text, its broken version): the checks must fail with each
NEGATIVE = [
    ("a frame rendered a group after its reserved start", "ultra_parlio.h",
     "      lane->cur_start_ = lane->p_start_;\n", "      lane->cur_start_ = lane->p_start_ + 1;\n"),
    ("no render deadline (the DMA plays a late chunk's buffer again)", "ultra_parlio.h",
     "  int64_t deadline_of_(int64_t k) const {\n    return ParlioClock",
     "  int64_t deadline_of_(int64_t k) const {\n    return 0 * ParlioClock"),
    ("a stop leaves the releases of the frames it cut armed", "ultra_parlio.h",
     "      this->release_us_[k] = 0;\n      this->lanes_[k].cut_at(", "      this->lanes_[k].cut_at("),
    ("a stop cancels the release of a frame that was out", "ultra_parlio.h",
     "      if (this->release_us_[k] != 0 && this->release_end_us_[k] <= cut_us)\n", "      if (false)\n"),
    ("a handover accepted after its deadline (as long as the timer hasn't stopped the stream yet)", "ultra_parlio.h",
     "    if (this->killed() || this->deadline_us_ == 0 || now_us >= this->deadline_us_)\n      return false;\n",
     "    if (this->killed())\n      return false;\n"),
    ("the deadlines counted from the clock read after the start's transmit call", "ultra_parlio.h",
     "    this->skew_ns_ = skew;\n", "    this->skew_ns_ = 0;\n"),
    ("a slow start not tried again", "ultra_parlio.h",
     "    if (skew > PARLIO_START_SKEW_NS && !any_skew)\n      return false;\n", ""),
    ("the lane's release writes the direction pin through the ISR pin's own allocation", "dig2analog_ultra.cpp",
     "    gpio_ll_set_level(&GPIO, hub->rw_dir_gpio_, hub->rw_dir_idle_);\n", "    hub->rw_dir_isr_.digital_write(false);\n"),
    ("an unsent poll frame keeps its UART turn when its port goes down", "dig2analog_ultra.cpp",
     "    if (this->tx_prepared_ && this->tx_rw_ && !this->rw_open_) {\n      this->tx_prepared_ = false;\n"
     "      this->tx_rw_ = false;\n      this->rw_release_ = false;\n      this->rw_drop_turn_();\n    }\n", ""),
    ("a lane's notice copy counted once its end passed, refused or cut", "dig2analog_ultra.cpp",
     "  const uint8_t outcome = this->lane_->outcome(this->notice_id_, this->notice_end_us_, now_us_());\n",
     "  const uint8_t outcome = now_us_() >= this->notice_end_us_ ? ParlioLane::ON_WIRE : ParlioLane::PENDING;\n"),
    ("the lane's release callback named without telemetry", "dig2analog_ultra.cpp",
     "#else\n    const ParlioLane::ReleaseFn release = nullptr;\n#endif\n",
     "#else\n    const ParlioLane::ReleaseFn release = UltraHub::rw_lane_release_;\n#endif\n"),
    ("a frame the stream refused counted as sent", "dig2analog_ultra.cpp",
     "    return this->lane_->take_refused() ? AGAIN : SENT;\n", "    return SENT;\n"),
    ("an expired reservation ends the session", "dig2analog_ultra.cpp",
     "        vTaskDelay(1);   // its start passed (or the stream restarted): a new one\n        continue;\n",
     "        return STOPPED;\n"),
    ("an abort doesn't end the wait for the stream", "dig2analog_ultra.cpp",
     "    for (uint32_t t = 0; this->lane_->busy(); t++) {\n      if (abortable && this->upd_abort_.load())\n"
     "        return ABORT;\n", "    for (uint32_t t = 0; this->lane_->busy(); t++) {\n"),
    ("the group start without a lane's frame cadence", "dig2analog_ultra.cpp",
     "  frame = std::max(frame, this->lane_lead_us_);\n#endif\n  if (this->start_delay_us_",
     "#endif\n  if (this->start_delay_us_"),
    ("the UART to whichever line asks first (no queue)", "dig2analog_ultra.cpp",
     "    if (hub->rw_waiting_ && now - hub->rw_wait_seen_ms_ < RW_WAIT_STALE_MS &&\n"
     "        static_cast<int32_t>(hub->rw_wait_order_ - this->rw_wait_order_) < 0)\n"
     "      return false;   // it has waited longer\n", ""),
    ("an update keeps the turn of a poll frame that never went out", "dig2analog_ultra.cpp",
     "  this->tx_rw_ = false;        // a poll frame that never went out: its UART's turn goes to the next line\n"
     "  this->rw_drop_turn_();\n", ""),
    ("a lane's notice copy reported to its port as it is posted (not as it starts on the wire)", "dig2analog_ultra.cpp",
     "      this->notice_id_ = this->lane_->posted_id();   // (once out on the wire the port is told: service_notice_())\n"
     "      this->notice_end_us_ = this->lane_->frame_end_us();\n",
     "      this->port_->notice_sent();\n"),
    ("the lock interval after a preamble on a lane waits for the RMT", "dig2analog_ultra.cpp",
     "    return now_us_() >= this->lane_->end_us();   // (its frame and its reset: the lane's quiet after it)\n",
     "    return rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK;\n"),
    ("the shutdown's notice on a lane waits for the RMT", "dig2analog_ultra.cpp",
     "  if (this->line_free_())\n    this->transmit_();\n}\n",
     "  if (rmt_tx_wait_all_done(this->channel_, 0) == ESP_OK)\n    this->transmit_();\n}\n"),
    ("no turn-taking on a shared UART", "dig2analog_ultra.cpp",
     "    if (hub->rw_turn_)\n      return false;   // its window is open\n", ""),
    ("the direction pin let go of a byte before the RW frame's end", "ultra_parlio.h",
     "const int64_t bit_start = (lane->cur_start_ + lane->cur_bytes_) * this->group_samples() - this->period_;",
     "const int64_t bit_start = (lane->cur_start_ + lane->cur_bytes_ - 1) * this->group_samples() - this->period_;"),
    ("the direction pin let go of 200 us after the RW frame's end", "ultra_parlio.h",
     "static constexpr uint32_t PARLIO_RELEASE_NS = 2000;", "static constexpr uint32_t PARLIO_RELEASE_NS = 200000;"),
]

FEATURES = ("USE_DIG2ANALOG_ULTRA_TELEMETRY", "USE_DIG2ANALOG_ULTRA_PARLIO", "USE_DIG2ANALOG_ULTRA_UPDATE",
            "USE_DIFFPOWER")


def read_sources(mutation=None) -> dict:
    sources = {name: (COMP / name).read_text(encoding="utf-8") for name in SOURCES}
    if mutation is not None:
        _, name, old, new = mutation
        assert sources[name].count(old) == 1, f"negative control: '{old.strip()[:60]}' not found once in {name}"
        sources[name] = sources[name].replace(old, new)
    return sources


def preprocess(text: str, defines, out: Path) -> str:
    src = out / "pp.cpp"
    src.write_text("".join(line for line in text.splitlines(keepends=True) if not line.lstrip().startswith("#include")),
                   encoding="utf-8")
    cmd = ZIG + ["c++", "-E", "-P", "-x", "c++", "-DUSE_ESP32", *(f"-D{d}" for d in defines), str(src)]
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def feature_check(sources: dict, out: Path) -> list:
    """For every combination of the features: the members of UltraHub the .cpp names are declared in the .h."""
    out.mkdir(parents=True, exist_ok=True)
    fails = []
    for on in product((False, True), repeat=len(FEATURES)):
        defines = [d for d, o in zip(FEATURES, on) if o]
        header = preprocess(sources["dig2analog_ultra.h"], defines, out)
        body = preprocess(sources["dig2analog_ultra.cpp"], defines, out)
        declared = set(re.findall(r"\b(\w+)\s*[({;=\[]", header)) | set(re.findall(r"\b(\w+)\b", header))
        used = set(re.findall(r"\bUltraHub::(\w+)", body)) | set(re.findall(r"\bthis->(\w+_)\b", body))
        missing = sorted(n for n in used if n not in declared)
        if missing:
            fails.append(f"FAIL feature check: with {' + '.join(d.split('_')[-1].lower() for d in defines) or 'no options'}, "
                         f"dig2analog_ultra.cpp names {', '.join(missing)}, not declared in dig2analog_ultra.h")
    return fails


def prepare(out: Path, sources: dict) -> None:
    """build/parlio/<variant>: the cut hub pieces and ultra_parlio.h."""
    out.mkdir(parents=True, exist_ok=True)
    for inc, signatures in PIECES.items():
        text = sources["dig2analog_ultra.cpp"]
        (out / inc).write_text("\n\n".join(definition(text, s) for s in signatures) + "\n", encoding="utf-8")
    (out / "port.inc").write_text("\n\n".join(definition(sources["../diffpower/diffpower.cpp"], s) for s in PORT_PIECES)
                                  + "\n", encoding="utf-8")
    (out / "ultra_parlio.h").write_text(sources["ultra_parlio.h"], encoding="utf-8")


def build(out: Path, flags) -> Path:
    exe = out / ("ultra_parlio_test.exe" if sys.platform == "win32" else "ultra_parlio_test")
    # the test's include of ultra_parlio.h comes from the variant's copy, its hub part from tests/
    for name in ("ultra_parlio_test.cpp", "ultra_parlio_test_hubs.inc"):
        text = (HERE / name).read_text(encoding="utf-8")
        (out / name).write_text(
            text.replace('#include "../components/dig2analog_ultra/ultra_parlio.h"', '#include "ultra_parlio.h"')
            .replace('#include "../components/', f'#include "{(ROOT / "components").as_posix()}/'), encoding="utf-8")
    cmd = ZIG + ["c++", "-std=c++17", "-Wall", "-Wno-unused-variable", "-Wno-unused-function",
                 "-Wno-nullability-completeness", f"-I{out}", *flags, str(out / "ultra_parlio_test.cpp"), "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return exe


def main() -> int:
    args = sys.argv[1:]
    checked = ["-O1", "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    base = BUILD / "test"
    sources = read_sources()
    fails = feature_check(sources, base)
    for line in fails:
        print(line, flush=True)
    print(f"feature check: the members dig2analog_ultra.cpp names are declared, with each of the "
          f"{2 ** len(FEATURES)} combinations of telemetry / PARLIO / updates / Diff-Power ports: {'FAILED' if fails else 'ok'}", flush=True)
    prepare(base, sources)
    rc = subprocess.run([str(build(base, checked))]).returncode
    if fails:
        rc = 1
    if rc != 0:
        return rc
    if "--no-negative" not in args:
        print("\nnegative controls (each must fail):", flush=True)
        only = args[args.index("--only") + 1] if "--only" in args else ""
        for i, m in enumerate(NEGATIVE):
            if only not in m[0]:
                continue
            out = BUILD / f"negative{i}"
            sources = read_sources(m)
            fails = feature_check(sources, out)
            prepare(out, sources)
            try:
                proc = subprocess.run([str(build(out, checked))], capture_output=True, text=True, timeout=900)
                fails += [line for line in proc.stdout.splitlines() if line.startswith("FAIL ")]
            except subprocess.TimeoutExpired:
                fails.append("FAIL: the test hung (900 s)")
            ok = bool(fails)
            print(f"  {'ok  ' if ok else 'MISS'} {m[0]}: {len(fails)} check(s) failed", flush=True)
            for line in fails[:4]:
                print(f"         {line.split(': ', 1)[-1][:150]}", flush=True)
            if not ok:
                rc = 1
    if "--no-bench" not in args:
        print("\nthe render's cost (-O2, this PC):", flush=True)
        out = BUILD / "bench"
        prepare(out, read_sources())
        subprocess.run([str(build(out, ["-O2", "-DNDEBUG"])), "--bench"], check=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
