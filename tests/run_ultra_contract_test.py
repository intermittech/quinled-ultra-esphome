"""Build and run tests/ultra_contract_test.cpp with the zig C/C++ compiler (pip install ziglang): the component's
scheduler against the board firmware's receive path (frame_rx.c, proto.c) and light engine (ultra.c). Needs the board
firmware (receiver_dependency.py)."""

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
    from run_wire3_contract_test import main as wire3_contract
    if wire3_contract(): return 1
    core = receiver_contract(firmware_dir())
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_contract_test.exe" if sys.platform == "win32" else "ultra_contract_test")
    objs = []
    for src in ("ultra.c", "ultra_tables.c", "frame_rx.c", "proto.c", "math32.c", "config.c", "presets.c"):
        obj = BUILD / ("contract_" + src[:-2] + ".o")
        subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                              "-c", str(core / src), "-o", str(obj)], check=True)
        objs.append(str(obj))
    print(f"board firmware: {core}")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-I{core}",
                          str(HERE / "ultra_contract_test.cpp"), *objs, "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
