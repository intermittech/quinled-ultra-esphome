"""Build and run tests/ultra_contract_test.cpp with the zig C/C++ compiler (pip install ziglang): the component's
scheduler against the board firmware's receive path (frame_rx.c, proto.c) and light engine (ultra.c). Needs the board
firmware: ULTRA_FIRMWARE=<firmware dir>, else a checkout next to this repository (see run_ultra_host_test.py)."""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def firmware_dir() -> Path:
    env = os.environ.get("ULTRA_FIRMWARE")
    if env:
        return Path(env)
    for cand in (ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"):
        if (cand / "src" / "core" / "ultra.c").exists():
            return cand
    return ROOT.parent / "quinled-ultra-firmware"


def main() -> int:
    core = firmware_dir() / "src" / "core"
    if not (core / "ultra.c").exists():
        print(f"board firmware not found ({core}): contract test skipped")
        return 0
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_contract_test.exe" if sys.platform == "win32" else "ultra_contract_test")
    objs = []
    for src in ("ultra.c", "ultra_tables.c", "frame_rx.c", "proto.c"):
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
