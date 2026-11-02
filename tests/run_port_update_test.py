"""Build and run tests/port_update_test.cpp with the zig C/C++ compiler (pip install ziglang): "update all receivers
on this port", its host side (port_inventory.h)."""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build" / "port_update"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("port_update_test.exe" if sys.platform == "win32" else "port_update_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", str(HERE / "port_update_test.cpp"),
                          "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
