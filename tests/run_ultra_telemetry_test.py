"""Build and run tests/ultra_telemetry_test.cpp with the zig C/C++ compiler (pip install ziglang): the return
channel's hub side (ultra_telemetry.h) against a modelled chain of Diff-Power ultra receivers. Needs nothing else; the
receiver firmware's own answer path is tests/run_ultra_telemetry_contract_test.py."""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_telemetry_test.exe" if sys.platform == "win32" else "ultra_telemetry_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", str(HERE / "ultra_telemetry_test.cpp"),
                          "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
