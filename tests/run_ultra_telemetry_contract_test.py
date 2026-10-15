"""Build and run tests/ultra_telemetry_contract_test.cpp with the zig C/C++ compiler (pip install ziglang): the
component's return-channel poller (ultra_telemetry.h) against the Diff-Power ultra receiver firmware's own receive /
answer path (BOARD=2: src/app/main.c on the firmware's hardware model, test/test_soak.c with test_soak_rc.c, through
tests/ultra_telemetry_receiver.c).

Needs the board firmware: ULTRA_FIRMWARE=<firmware dir>, else a checkout next to this repository
(../dig2analog-ultra-5ch/firmware or ../quinled-ultra-firmware). Without it the test is skipped, unless
ULTRA_REQUIRE_DEPS=1 (CI): then that is a failure. Everything it builds goes to tests/build/telemetry; the firmware
tree is only read."""

from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "telemetry"
ZIG = [sys.executable, "-m", "ziglang"]
# the receiver as test/build_host.cmd builds its soak (soak_rc_tests.exe): BOARD=2, no MON_EXTRA, the full console with
# 0.6.4 layout's texts (genlow), the 0.2.4 pages
DEFINES = ["-DUPD_CRC_ULTRA", "-DCONSOLE_FULL=1", "-DBOARD=2", "-DMON_EXTRA=0", "-DRC_PAGES_024=1"]
# the firmware's C as it is (MSVC /W4 there): no warnings asked for; every variable in data (power-ups restore them)
C_FLAGS = ["-std=gnu99", "-O1", "-fno-zero-initialized-in-bss", "-fsanitize=undefined", "-fno-sanitize-recover=all",
           *DEFINES]
# src/core/cli.c's cz_at(z) is `cz_text + CZ_BIAS + (z)`: a pointer 2048 bytes past the console's text blob before the
# (negative) offset brings it back. Harmless on the chip, but UB, and UBSan's array-bounds check stops there.
C_FILE_FLAGS = {"cli.c": ["-fno-sanitize=array-bounds"]}
CXX_FLAGS = ["-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
             "-fsanitize=undefined", "-fno-sanitize-recover=all"]
NEEDED = ["test/test_soak.c", "test/test_soak_rc.c", "test/soak/ch32v20x.h", "src/app/main.c",
          "tools/gen_console_text.py"]


def firmware_dir() -> Path:
    env = os.environ.get("ULTRA_FIRMWARE")
    if env:
        return Path(env)
    for cand in (ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"):
        if (cand / "test" / "test_soak_rc.c").exists():
            return cand
    return ROOT.parent / "quinled-ultra-firmware"


def run(cmd: list) -> None:
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout or r.stderr:
        sys.stdout.write(r.stdout + r.stderr)
    if r.returncode != 0:
        raise SystemExit(f"failed ({r.returncode}): {' '.join(str(c) for c in cmd)}")


def main() -> int:
    fw = firmware_dir().resolve()
    missing = [n for n in NEEDED if not (fw / n).exists()]
    if missing:
        if os.environ.get("ULTRA_REQUIRE_DEPS") == "1":
            print(f"receiver firmware not found ({fw}, no {missing[0]}): telemetry contract test FAILED "
                  "(ULTRA_REQUIRE_DEPS=1)")
            return 1
        print(f"receiver firmware not found ({fw}): telemetry contract test skipped")
        return 0
    print(f"receiver firmware: {fw}", flush=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    # the console's compressed texts (core/cz.h), as build_host.cmd's genlow (-B: no __pycache__ in the firmware tree)
    run([sys.executable, "-B", str(fw / "tools" / "gen_console_text.py"), str(BUILD / "genlow"), "--full"])
    inc = [f"-I{fw / 'test'}", f"-I{fw / 'test' / 'soak'}", f"-I{BUILD / 'genlow'}", f"-I{HERE}"]
    jobs = [(ZIG + ["cc", *C_FLAGS, *inc, "-c", str(HERE / "ultra_telemetry_receiver.c"), "-o",
                    str(BUILD / "receiver.o")], BUILD / "receiver.o")]
    for src in sorted((fw / "src" / "core").glob("*.c")):
        obj = BUILD / f"core_{src.stem}.o"
        jobs.append((ZIG + ["cc", *C_FLAGS, *C_FILE_FLAGS.get(src.name, []), *inc, "-DUCRX_FW_SECTION", "-include",
                            str(HERE / "ultra_telemetry_receiver.h"), "-c", str(src), "-o", str(obj)], obj))
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        list(pool.map(lambda j: run(j[0]), jobs))
    exe = BUILD / ("ultra_telemetry_contract_test.exe" if sys.platform == "win32" else "ultra_telemetry_contract_test")
    # (the month scenario calls src/core/energy.c itself: its header and what it includes)
    run(ZIG + ["c++", *CXX_FLAGS, f"-I{HERE}", f"-I{fw / 'src' / 'core'}", f"-I{fw / 'test' / 'soak'}",
               str(HERE / "ultra_telemetry_contract_test.cpp"), *[str(j[1]) for j in jobs], "-o", str(exe)])
    sys.stdout.flush()
    return subprocess.run([str(exe), *sys.argv[1:]]).returncode


if __name__ == "__main__":
    sys.exit(main())
