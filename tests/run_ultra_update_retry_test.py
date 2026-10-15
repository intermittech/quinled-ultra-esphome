"""Build and run tests/ultra_update_retry_test.cpp with the zig C/C++ compiler (pip install ziglang): auto_retry
(update_retry.h) with the component's update sender through the board firmware's bootloader and its flash / board
model (bootloader/test/test_bl.c, as the firmware's sender_tests use it).

Needs the board firmware: ULTRA_FIRMWARE=<firmware dir>, else a checkout next to this repository (as the host test
finds it). Without it the test is skipped; with ULTRA_REQUIRE_DEPS=1 (CI) that is a failure. The component's copy of
ultra_update.h must equal the firmware's.
"""

import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "update_retry"
ZIG = [sys.executable, "-m", "ziglang"]
COMP = ROOT / "components" / "dig2analog_ultra"

sys.path.insert(0, str(HERE))
from run_ultra_host_test import firmware_dir  # noqa: E402


def main() -> int:
    fw = firmware_dir()
    bl = fw / "bootloader"
    if not (bl / "test" / "test_bl.c").exists():
        if os.environ.get("ULTRA_REQUIRE_DEPS") == "1":
            print(f"FAIL: the board firmware's bootloader tests are missing ({bl}; ULTRA_REQUIRE_DEPS=1)")
            return 1
        print(f"board firmware not found ({bl}): the update retry test skipped")
        return 0
    def body(path: Path) -> bytes:
        """The header without a leading licence comment (the published firmware adds an SPDX one)."""
        text = path.read_bytes().replace(b"\r\n", b"\n")
        if text.startswith(b"/* SPDX-License-Identifier:"):
            text = text[text.index(b"*/\n") + 3:]
        return text

    if body(COMP / "ultra_update.h") != body(fw / "src" / "core" / "ultra_update.h"):
        print("FAIL: components/dig2analog_ultra/ultra_update.h differs from the firmware's src/core/ultra_update.h")
        return 1
    BUILD.mkdir(parents=True, exist_ok=True)
    objs = []
    for src in ("bl_core.c", "bl_rx.c"):
        obj = BUILD / (src[:-2] + ".o")
        subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                              f"-I{fw / 'src' / 'core'}", "-c", str(bl / src), "-o", str(obj)], check=True)
        objs.append(str(obj))
    exe = BUILD / ("ultra_update_retry_test.exe" if sys.platform == "win32" else "ultra_update_retry_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
                          "-Wno-nullability-completeness", "-Wno-deprecated", "-fsanitize=undefined",
                          "-fno-sanitize-recover=all", f"-I{bl / 'test'}", f"-I{bl}", f"-I{COMP}",
                          f"-I{fw / 'src' / 'core'}", str(HERE / "ultra_update_retry_test.cpp"), *objs, "-o", str(exe)],
                   check=True)
    print(f"board firmware: {fw}")
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
