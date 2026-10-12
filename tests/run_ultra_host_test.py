"""Build and run tests/ultra_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

With the QuinLED dig2analog-ultra-5ch firmware next to this repo (quinled-ultra-firmware, or
ULTRA_FIRMWARE=<firmware dir>), the board's light engine is compiled in and the chain test runs too; the protocol
header copy is checked against the firmware's master (and reported against a quinled-ultra-protocol checkout next to
this repo).
"""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def firmware_dir() -> Path:
    """The board firmware for the engine tests: $ULTRA_FIRMWARE, else a checkout next to this repository: the
    hardware project's dig2analog-ultra-5ch/firmware (where it is developed) or quinled-ultra-firmware."""
    env = os.environ.get("ULTRA_FIRMWARE")
    if env:
        return Path(env)
    for cand in (ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"):
        if (cand / "src" / "core" / "ultra.c").exists():
            return cand
    return ROOT.parent / "quinled-ultra-firmware"


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_host_test.exe" if sys.platform == "win32" else "ultra_host_test")
    fw = firmware_dir()
    core = fw / "src" / "core"
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    copy = (ROOT / "components" / "dig2analog_ultra" / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
    proto = ROOT.parent / "quinled-ultra-protocol" / "include" / "ultra_protocol.h"
    if proto.exists():
        same = proto.read_bytes().replace(b"\r\n", b"\n") == copy
        print(f"ultra_protocol.h {'matches' if same else 'DIFFERS from'} {proto} (the checked-out tag)")
    if (core / "ultra.c").exists():
        master = (core / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
        if master != copy:
            print("FAIL: components/dig2analog_ultra/ultra_protocol.h differs from the firmware's master copy")
            return 1
        for src in ("ultra.c", "ultra_tables.c", "config.c", "presets.c"):
            obj = BUILD / (src[:-2] + ".o")
            subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                                  "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
            objs.append(str(obj))
        cmd += ["-DWITH_ENGINE", f"-I{core}"]
        print(f"board engine: {core}")
    cmd += [str(HERE / "ultra_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
