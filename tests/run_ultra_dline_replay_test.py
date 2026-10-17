"""The ESPHome hub's digital port on the QuinLED Diff-Power Ultra-Digital receiver's own delay line.

tests/ultra_digital_transport_test.cpp runs the hub's digital-port methods (cut unchanged from dig2analog_ultra.cpp) with
a stand-in for the RMT driver and writes what the hub put on the wire (the RMT symbols, when the direction pin let go of
the line) to build/transport/traces/*.trace. This script replays those traces into the firmware's sample-exact model of
the receiver's hardware (test/test_dline.c with core/dline.c on it), with the model's own hub replaced by the replay (the
firmware's file is patched as text, not modified): every pixel frame must reach the strip whole and in order, nothing
else may (no marker, frame, release or answer), every poll must be answered, every capture must decode to the frame sent.

Needs the firmware (ULTRA_FIRMWARE, or a checkout next to this repository); skipped without it, a failure with
ULTRA_REQUIRE_DEPS=1. Usage: python tests/run_ultra_dline_replay_test.py
"""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "transport"
ZIG = [sys.executable, "-m", "ziglang"]


def firmware_core() -> Path | None:
    env = os.environ.get("ULTRA_FIRMWARE")
    cands = [Path(env)] if env else [ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"]
    for c in cands:
        core = c / "src" / "core"
        if (core / "dline.c").exists() and (core / "ws_decode.c").exists() and (c / "test" / "test_dline.c").exists():
            return c
    return None


def main() -> int:
    strict = os.environ.get("ULTRA_REQUIRE_DEPS") == "1"
    fw = firmware_core()
    if fw is None:
        print("the firmware (test/test_dline.c, core/dline.c) not found: the dline replay skipped")
        return 1 if strict else 0
    traces = BUILD / "traces"
    traces.mkdir(parents=True, exist_ok=True)
    for old in traces.glob("*.trace"):
        old.unlink()
    # 1. the hub's output: the transport test writes the traces
    exe = BUILD / ("ultra_digital_transport_test.exe" if sys.platform == "win32" else "ultra_digital_transport_test")
    if not exe.exists():
        print("run tests/run_ultra_digital_test.py first (it builds the transport test)")
        return 1
    env = dict(os.environ, ULTRA_TRANSPORT_TRACE=str(traces))
    run = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
    if run.returncode != 0:
        sys.stdout.write(run.stdout)
        return 1
    files = sorted(traces.glob("*.trace"))
    if not files:
        print("FAIL: the transport test wrote no trace")
        return 1
    # 2. the firmware's model, with the replay as its hub
    text = (fw / "test" / "test_dline.c").read_text(encoding="utf-8").replace("\r\n", "\n")
    edits = (
        ('#include "../src/core/', '#include "'),
        ("int main(int argc, char **argv)", "static int fw_main(int argc, char **argv)"),
        ("static void run(const scen_t *sc)\n", '#include "dline_replay_hook.inc"\n\nstatic void run(const scen_t *sc)\n'),
    )
    for old, new in edits:
        if old not in text:
            print(f"FAIL: the firmware's test_dline.c has changed ({old!r} not found): update tests/run_ultra_dline_replay_test.py")
            return 1
        text = text.replace(old, new)
    text += '\n#include "dline_replay_main.inc"\n'
    src = BUILD / "dline_replay.c"
    src.write_text(text, encoding="utf-8", newline="\n")
    core = fw / "src" / "core"
    out = BUILD / ("dline_replay.exe" if sys.platform == "win32" else "dline_replay")
    subprocess.run(ZIG + ["cc", "-std=gnu99", "-O2", "-w", f"-I{core}", f"-I{HERE}", str(src), str(core / "dline.c"),
                          str(core / "ws_decode.c"), "-lm", "-o", str(out)], check=True)
    args = [str(out)] + [f.as_posix() for f in files]
    run = subprocess.run(args, capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    # the replay has teeth: with the LOW after every marker frame 290 us short, the model finds it
    bad = subprocess.run(args, capture_output=True, text=True, env=dict(os.environ, ULTRA_REPLAY_MUTATE="1"))
    caught = bad.returncode != 0
    print(f"{'ok  ' if caught else 'FAIL'} negative control: the LOW after the marker frames 290 us short (10 us left) is {'found' if caught else 'NOT found'}")
    return run.returncode or (0 if caught else 1)


if __name__ == "__main__":
    sys.exit(main())
