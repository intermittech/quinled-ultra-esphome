"""Build and run tests/ultra_telemetry_contract_test.cpp with the zig C/C++ compiler (pip install ziglang): the
component's return-channel poller (ultra_telemetry.h) against the Diff-Power Ultra Receiver Analog-Medium firmware's own receive /
answer path (BOARD=2: src/app/main.c on the firmware's hardware model, test/test_soak.c with test_soak_rc.c, through
tests/ultra_telemetry_receiver.c).

Needs the board firmware (receiver_dependency.py). Everything it builds goes to tests/build/telemetry; the firmware
tree is only read."""

from concurrent.futures import ThreadPoolExecutor
import os
from receiver_dependency import firmware_dir, receiver_contract
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "telemetry"
ZIG = [sys.executable, "-m", "ziglang"]
# The release app on the receiver's soak model: BOARD=2, full console, no MON_EXTRA.
DEFINES = ["-DUPD_CRC_ULTRA", "-DUPD_APP=1", "-DCONSOLE_FULL=1", "-DBOARD=2", "-DMON_EXTRA=0"]
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


def run(cmd: list) -> None:
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout or r.stderr:
        sys.stdout.write(r.stdout + r.stderr)
    if r.returncode != 0:
        raise SystemExit(f"failed ({r.returncode}): {' '.join(str(c) for c in cmd)}")


def main() -> int:
    receiver_contract(firmware_dir())
    fw = firmware_dir().resolve()
    missing = [n for n in NEEDED if not (fw / n).exists()]
    if missing:
        print(f"FAIL: the receiver firmware ({fw}) has no {missing[0]}")
        return 1
    print(f"receiver firmware: {fw}", flush=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    # Read the firmware's generated input facts and raw descriptors, for all three receiver builds.
    descriptors = []
    for name, board, v407 in (("analog", "ur_analog_m", 0), ("digital203", "ur_digital_m", 0),
                              ("digital407", "ur_digital_m", 1)):
        header = (fw / "src/board" / board / "descriptor_gen.h").as_posix()
        descriptors.append(f'namespace bound_{name} {{\n#define MCU_CH32V40X {v407}\n#include "{header}"\n'
                           'static constexpr uint32_t num = BOARD_IN_WH_NUM, den = BOARD_IN_WH_DEN;\n}\n'
                           '#undef MCU_CH32V40X\n#undef BOARD_IN_WH_NUM\n#undef BOARD_IN_WH_DEN\n'
                           '#undef BOARD_DESCRIPTOR_BYTES\n#undef BOARD_DESCRIPTOR_CRC\n')
    (BUILD / "board_bound_descriptors.inc").write_text("\n".join(descriptors), encoding="utf-8")
    # the console's compressed texts (core/cz.h), as build_host.cmd's genlow (-B: no __pycache__ in the firmware tree)
    run([sys.executable, "-B", str(fw / "tools" / "gen_console_text.py"), str(BUILD / "genlow"), "--full"])
    inc = [f"-I{fw / 'test'}", f"-I{fw / 'test' / 'soak'}", f"-I{fw / 'src' / 'mcu' / 'ch32v20x'}",
           f"-I{BUILD / 'genlow'}", f"-I{HERE}"]
    jobs = [(ZIG + ["cc", *C_FLAGS, *inc, "-c", str(HERE / "ultra_telemetry_receiver.c"), "-o",
                    str(BUILD / "receiver.o")], BUILD / "receiver.o")]
    for src in sorted((fw / "src" / "core").glob("*.c")):
        obj = BUILD / f"core_{src.stem}.o"
        jobs.append((ZIG + ["cc", *C_FLAGS, *C_FILE_FLAGS.get(src.name, []), *inc, "-DUCRX_FW_SECTION", "-include",
                            str(HERE / "ultra_telemetry_receiver.h"), "-c", str(src), "-o", str(obj)], obj))
    with ThreadPoolExecutor(max_workers=max(1, min(4, int(os.environ.get("ULTRA_TEST_JOBS", "4")), os.cpu_count() or 4))) as pool:
        list(pool.map(lambda j: run(j[0]), jobs))
    exe = BUILD / ("ultra_telemetry_contract_test.exe" if sys.platform == "win32" else "ultra_telemetry_contract_test")
    # (the month scenario calls src/core/energy.c itself: its header and what it includes)
    run(ZIG + ["c++", *CXX_FLAGS, f"-I{HERE}", f"-I{BUILD}", f"-I{fw / 'src' / 'core'}", f"-I{fw / 'test' / 'soak'}",
               str(HERE / "ultra_telemetry_contract_test.cpp"), *[str(j[1]) for j in jobs], "-o", str(exe)])
    sys.stdout.flush()
    return subprocess.run([str(exe), *sys.argv[1:]]).returncode


if __name__ == "__main__":
    sys.exit(main())
