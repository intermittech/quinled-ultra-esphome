"""Build and run tests/ultra_telemetry_fuzz.cpp (the return channel's hub side fed random, malformed and boundary
byte streams; UBSan) with the zig C++ compiler (pip install ziglang).

Usage: python tests/run_ultra_telemetry_fuzz.py [seeds] [windows per seed]   (default 60 x 1500, a few seconds)
"""
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
COMP = HERE.parent / "components" / "ultra_line"
BUILD = HERE / "build"


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_telemetry_fuzz.exe" if sys.platform == "win32" else "ultra_telemetry_fuzz")
    subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                    f"-I{COMP}", str(HERE / "ultra_telemetry_fuzz.cpp"), "-o", str(exe)], check=True, timeout=600)
    args = sys.argv[1:] or ["60", "1500"]
    return subprocess.run([str(exe), *args], timeout=600).returncode


if __name__ == "__main__":
    sys.exit(main())
