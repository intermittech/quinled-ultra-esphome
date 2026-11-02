"""Build and run tests/diffpower_test.cpp with the zig C/C++ compiler (pip install ziglang): the Diff-Power port
power manager's core (components/diffpower/diffpower_core.h) with simulated time. Needs nothing else."""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("diffpower_test.exe" if sys.platform == "win32" else "diffpower_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", str(HERE / "diffpower_test.cpp"),
                          "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
