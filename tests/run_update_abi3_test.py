"""Build ABI-3 updater unit tests with the repository host toolchain."""
from pathlib import Path
import subprocess
import sys
HERE = Path(__file__).resolve().parent
BUILD = HERE / "build" / "update_abi3"
def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("update_abi3_test.exe" if sys.platform == "win32" else "update_abi3_test")
    subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                    "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                    str(HERE / "update_abi3_test.cpp"), "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode
if __name__ == "__main__":
    sys.exit(main())
