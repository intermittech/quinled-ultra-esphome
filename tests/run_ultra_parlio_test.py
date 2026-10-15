"""Build and run tests/ultra_parlio_test.cpp with the zig C/C++ compiler (pip install ziglang): the PARLIO lanes
(ultra_parlio.h) and the telemetry coordinator for lines sharing a UART, with the hub's own loop / transmit / window
code cut unchanged from components/dig2analog_ultra/dig2analog_ultra.cpp (as run_ultra_lifecycle_test.py does).

Then the negative controls: the same test against copies of the sources with one thing broken (a frame rendered a
group late, the turn-taking on a shared UART gone, the direction pin let go of a byte early or 200 us late); each
must fail. And the render's cost per chunk (-O2, no sanitizers).

Usage: python tests/run_ultra_parlio_test.py [--no-negative] [--no-bench]
"""

from pathlib import Path
import shutil
import subprocess
import sys

from run_ultra_lifecycle_test import definition

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
COMP = ROOT / "components" / "dig2analog_ultra"
BUILD = HERE / "build" / "parlio"
ZIG = [sys.executable, "-m", "ziglang"]

# <name>.inc: the definitions in dig2analog_ultra.cpp that start with these signatures
PIECES = {
    "hub_loop.inc": ["void UltraHub::loop("],
    "hub_transmit.inc": ["void UltraHub::transmit_(", "esp_err_t UltraHub::lane_post_("],
    "hub_rw.inc": [
        "bool UltraHub::rw_due_(", "bool UltraHub::rw_uart_free_(", "bool IRAM_ATTR UltraHub::rw_done_isr_(",
        "void IRAM_ATTR UltraHub::rw_lane_release_(", "void UltraHub::finish_window_(", "void UltraHub::rw_publish_(",
    ],
    "hub_update_lane.inc": ["uint8_t UltraHub::update_lane_("],
}

# (what breaks, file, the text, its broken version): the test must fail with each
NEGATIVE = [
    ("a frame rendered a group after its reserved start", "ultra_parlio.h",
     "      lane->cur_start_ = lane->p_start_;\n", "      lane->cur_start_ = lane->p_start_ + 1;\n"),
    ("no turn-taking on a shared UART", "dig2analog_ultra.cpp",
     "  if (urgent || this->rw_open_ || !this->rw_uart_free_())\n", "  if (urgent || this->rw_open_)\n"),
    ("the direction pin let go of a byte before the RW frame's end", "ultra_parlio.h",
     "const int64_t bit_start = (lane->cur_start_ + lane->cur_bytes_) * this->group_samples() - this->period_;",
     "const int64_t bit_start = (lane->cur_start_ + lane->cur_bytes_ - 1) * this->group_samples() - this->period_;"),
    ("the direction pin let go of 200 us after the RW frame's end", "ultra_parlio.h",
     "static constexpr uint32_t PARLIO_RELEASE_NS = 2000;", "static constexpr uint32_t PARLIO_RELEASE_NS = 200000;"),
]


def prepare(out: Path, mutation=None) -> None:
    """build/parlio/<variant>: the cut hub pieces and ultra_parlio.h, with one mutation applied."""
    out.mkdir(parents=True, exist_ok=True)
    sources = {name: (COMP / name).read_text(encoding="utf-8") for name in ("dig2analog_ultra.cpp", "ultra_parlio.h")}
    if mutation is not None:
        _, name, old, new = mutation
        assert sources[name].count(old) == 1, f"negative control: '{old.strip()}' not found once in {name}"
        sources[name] = sources[name].replace(old, new)
    for inc, signatures in PIECES.items():
        text = sources["dig2analog_ultra.cpp"]
        (out / inc).write_text("\n\n".join(definition(text, s) for s in signatures) + "\n", encoding="utf-8")
    (out / "ultra_parlio.h").write_text(sources["ultra_parlio.h"], encoding="utf-8")


def build(out: Path, flags) -> Path:
    exe = out / ("ultra_parlio_test.exe" if sys.platform == "win32" else "ultra_parlio_test")
    test = (HERE / "ultra_parlio_test.cpp").read_text(encoding="utf-8")
    # the test's include of ultra_parlio.h comes from the variant's copy
    src = out / "ultra_parlio_test.cpp"
    src.write_text(test.replace('#include "../components/dig2analog_ultra/ultra_parlio.h"', '#include "ultra_parlio.h"')
                   .replace('#include "../components/', f'#include "{(ROOT / "components").as_posix()}/'),
                   encoding="utf-8")
    cmd = ZIG + ["c++", "-std=c++17", "-Wall", "-Wno-unused-variable", "-Wno-unused-function",
                 "-Wno-nullability-completeness", f"-I{out}", *flags, str(src), "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return exe


def main() -> int:
    args = sys.argv[1:]
    checked = ["-O1", "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    base = BUILD / "test"
    prepare(base)
    rc = subprocess.run([str(build(base, checked))]).returncode
    if rc != 0:
        return rc
    if "--no-negative" not in args:
        print("\nnegative controls (each must fail):", flush=True)
        for i, m in enumerate(NEGATIVE):
            out = BUILD / f"negative{i}"
            prepare(out, m)
            proc = subprocess.run([str(build(out, checked))], capture_output=True, text=True)
            fails = [line for line in proc.stdout.splitlines() if line.startswith("FAIL ")]
            ok = proc.returncode != 0 and fails
            print(f"  {'ok  ' if ok else 'MISS'} {m[0]}: {len(fails)} check(s) failed", flush=True)
            for line in fails[:4]:
                print(f"         {line.split(': ', 1)[-1][:150]}", flush=True)
            if not ok:
                rc = 1
    if "--no-bench" not in args:
        print("\nthe render's cost (-O2, this PC):", flush=True)
        out = BUILD / "bench"
        prepare(out)
        subprocess.run([str(build(out, ["-O2", "-DNDEBUG"])), "--bench"], check=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
