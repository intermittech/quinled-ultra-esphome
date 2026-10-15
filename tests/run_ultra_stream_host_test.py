"""Build and run tests/ultra_stream_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

With the QuinLED Dig2Analog-Ultra-5ch firmware next to this repo (quinled-ultra-firmware, or
ULTRA_FIRMWARE=<firmware dir>), the board's light engine is compiled in and the glide test runs on it too. Without
it that part is skipped; with ULTRA_REQUIRE_DEPS=1 (CI) a missing firmware is a failure.
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
    exe = BUILD / ("ultra_stream_host_test.exe" if sys.platform == "win32" else "ultra_stream_host_test")
    fw = firmware_dir()
    core = fw / "src" / "core"
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    if (core / "ultra.c").exists():
        for src in ("ultra.c", "ultra_tables.c"):
            obj = BUILD / ("stream_" + src[:-2] + ".o")
            subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                                  "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
            objs.append(str(obj))
        cmd += ["-DWITH_ENGINE", f"-I{core}"]
        print(f"board engine: {core}")
    elif os.environ.get("ULTRA_REQUIRE_DEPS") == "1":
        print(f"FAIL: the board firmware is missing ({core}; ULTRA_REQUIRE_DEPS=1)")
        return 1
    else:
        print(f"board firmware not found ({core}): the engine part skipped")
    cmd += [str(HERE / "ultra_stream_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
