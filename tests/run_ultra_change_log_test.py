"""Build and run tests/ultra_change_log_test.cpp (the log of light changes, components/ultra_line/ultra_change_log.h)
with the zig C/C++ compiler (pip install ziglang). The lines it saw: tests/build/change_log/transcript.txt.
"""
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build" / "change_log"


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("ultra_change_log_test.exe" if sys.platform == "win32" else "ultra_change_log_test")
    subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                    str(HERE / "ultra_change_log_test.cpp"), "-o", str(exe)], check=True, timeout=600)
    return subprocess.run([str(exe), str(BUILD / "transcript.txt")], timeout=300).returncode


if __name__ == "__main__":
    sys.exit(main())
