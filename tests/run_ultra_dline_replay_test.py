"""The ESPHome hub's digital port on the Diff-Power Ultra Receiver Digital-Medium's own delay line.

tests/ur_digital_m_transport_test.cpp runs the hub's digital-port methods (cut unchanged from ultra_line.cpp) with
a stand-in for the RMT driver and writes what the hub put on the wire (the RMT symbols, when the direction pin let go of
the line) to build/transport/traces/*.trace. This script replays those traces into the firmware's sample-exact model of
the receiver's hardware (test/test_dline.c with core/dline.c on it), with the model's own hub replaced by the replay (the
firmware's file is patched as text, not modified): every pixel frame must reach the strip whole and in order, nothing
else may (no marker, frame, release or answer), every poll must be answered, every capture must decode to the frame sent.

Needs the firmware (receiver_dependency.py). Usage: python tests/run_ultra_dline_replay_test.py
"""

import os
from receiver_dependency import firmware_dir, receiver_contract
from pathlib import Path
import subprocess
import sys
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "transport"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    fw = firmware_dir()
    receiver_contract(fw)
    if not all((fw / f).exists() for f in ("src/core/dline.c", "src/core/ws_decode.c", "test/test_dline.c")):
        print(f"FAIL: the firmware ({fw}) has no test/test_dline.c, core/dline.c or core/ws_decode.c")
        return 1
    traces = BUILD / "traces"
    traces.mkdir(parents=True, exist_ok=True)
    for old in traces.glob("*.trace"):
        old.unlink()
    # 1. the hub's output: the transport test writes the traces
    exe = BUILD / ("ur_digital_m_transport_test.exe" if sys.platform == "win32" else "ur_digital_m_transport_test")
    if not exe.exists():
        print("run tests/run_ur_digital_m_test.py first (it builds the transport test)")
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
    # An exact legal boundary passes; both a genuinely short LOW and one RMT tick below the minimum must fail
    # the physical wire assertion specifically. An unrelated crash or a mutation with no applicable pair cannot pass.
    boundary = subprocess.run(args, capture_output=True, text=True,
                              env=dict(os.environ, ULTRA_REPLAY_MUTATE="boundary"))
    boundary_ok = boundary.returncode == 0
    print(f"{'ok  ' if boundary_ok else 'FAIL'} positive control: exact protocol-minimum LOW after marker frames")
    if not boundary_ok:
        sys.stdout.write(boundary.stdout)
        sys.stderr.write(boundary.stderr)
    caught = True
    for mode, label in (("1", "10 us LOW"), ("one-tick-short", "one RMT tick below the protocol minimum")):
        bad = subprocess.run(args, capture_output=True, text=True, env=dict(os.environ, ULTRA_REPLAY_MUTATE=mode))
        gaps = re.findall(r"physical LOW after marker frame: (\d+) RMT ticks, need >= (\d+)", bad.stdout)
        found = bad.returncode == 1 and any(int(gap) == 800 if mode == "1" else int(need) - int(gap) == 1
                                           for gap, need in gaps)
        caught &= found
        print(f"{'ok  ' if found else 'FAIL'} negative control: {label} is {'found' if found else 'NOT found'}")
        if not found:
            sys.stdout.write(bad.stdout)
            sys.stderr.write(bad.stderr)
    return run.returncode or (0 if caught and boundary_ok else 1)


if __name__ == "__main__":
    sys.exit(main())
