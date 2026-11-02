"""Build and run tests/ultra_stream_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

The board's light engine (the receiver firmware, receiver_dependency.py) is compiled in and the glide test runs on it.
"""

import os
from receiver_dependency import firmware_dir, receiver_contract
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_stream_host_test.exe" if sys.platform == "win32" else "ultra_stream_host_test")
    core = receiver_contract(firmware_dir())
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    for src in ("ultra.c", "ultra_tables.c", "math32.c", "config.c", "presets.c"):
        obj = BUILD / ("stream_" + src[:-2] + ".o")
        subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                              "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
        objs.append(str(obj))
    cmd += ["-DWITH_ENGINE", f"-I{core}"]
    print(f"board engine: {core}")
    cmd += [str(HERE / "ultra_stream_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
