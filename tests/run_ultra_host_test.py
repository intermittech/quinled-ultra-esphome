"""Build and run tests/ultra_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

With the QuinLED dig2analog-ultra-5ch firmware next to this repo (quinled-ultra-firmware, or
ULTRA_FIRMWARE=<firmware dir>), the board's light engine is compiled in and the chain test runs too; the protocol
header copy must equal the firmware's master and a quinled-ultra-protocol checkout next to this repo (the tag CI pins).
Without them the parts that need them are skipped; with ULTRA_REQUIRE_DEPS=1 (CI) a missing one is a failure.
"""

import json
import os
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def firmware_dir() -> Path:
    """The board firmware for the engine tests: $ULTRA_FIRMWARE, else a checkout next to this repository: the
    hardware project's dig2analog-ultra-5ch/firmware (where it is developed) or quinled-ultra-firmware."""
    env = os.environ.get("ULTRA_FIRMWARE")
    if env:
        return Path(env)
    for cand in (ROOT.parent / "dig2analog-ultra-5ch" / "firmware", ROOT.parent / "quinled-ultra-firmware"):
        if (cand / "src" / "core" / "ultra.c").exists():
            return cand
    return ROOT.parent / "quinled-ultra-firmware"


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_host_test.exe" if sys.platform == "win32" else "ultra_host_test")
    fw = firmware_dir()
    core = fw / "src" / "core"
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    strict = os.environ.get("ULTRA_REQUIRE_DEPS") == "1"
    copy = (ROOT / "components" / "dig2analog_ultra" / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
    proto = ROOT.parent / "quinled-ultra-protocol" / "include" / "ultra_protocol.h"
    if proto.exists():
        # the header as checked out (git HEAD: CI's tag); a local checkout may hold uncommitted work on the next one
        tag, master = "the checked-out tag", proto.read_bytes()
        git = ["git", "-C", str(proto.parent.parent)]
        head = subprocess.run(git + ["show", "HEAD:include/ultra_protocol.h"], capture_output=True)
        if head.returncode == 0:
            master = head.stdout
            desc = subprocess.run(git + ["describe", "--tags", "--always"], capture_output=True, text=True)
            tag = desc.stdout.strip() or tag
        if master.replace(b"\r\n", b"\n") == copy:
            print(f"ultra_protocol.h matches {proto} ({tag})")
        elif not strict and proto.read_bytes().replace(b"\r\n", b"\n") == copy:
            # the next release, not committed yet: matches the working tree (CI checks out a tag: HEAD only)
            print(f"ultra_protocol.h matches {proto} as checked out, uncommitted (not {tag})")
        else:
            print(f"FAIL: components/dig2analog_ultra/ultra_protocol.h differs from {proto} ({tag})")
            return 1
    elif strict:
        print(f"FAIL: the protocol repository is missing ({proto}; ULTRA_REQUIRE_DEPS=1)")
        return 1
    else:
        print(f"quinled-ultra-protocol not found ({proto}): the header and test vector checks skipped")
    # the test's copy of protocol test vectors (VEC_<NAME> = "<hex>") against the protocol repository's vectors.json
    vectors = ROOT.parent / "quinled-ultra-protocol" / "test-vectors" / "vectors.json"
    if not vectors.exists() and strict:
        print(f"FAIL: the protocol's test vectors are missing ({vectors}; ULTRA_REQUIRE_DEPS=1)")
        return 1
    if vectors.exists():
        known = {v["name"]: v["hex"] for v in json.loads(vectors.read_text(encoding="utf-8"))["vectors"]}
        src = (HERE / "ultra_host_test.cpp").read_text(encoding="utf-8")
        for name, hexstr in re.findall(r'VEC_([A-Z0-9_]+) =\s*"([0-9a-f]+)"', src):
            if known.get(name.lower()) != hexstr:
                print(f"FAIL: test vector {name.lower()} differs from {vectors}")
                return 1
            print(f"test vector {name.lower()} matches {vectors.name}")
    if (core / "ultra.c").exists():
        master = (core / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
        if master != copy:
            print("FAIL: components/dig2analog_ultra/ultra_protocol.h differs from the firmware's master copy")
            return 1
        for src in ("ultra.c", "ultra_tables.c", "config.c", "presets.c"):
            obj = BUILD / (src[:-2] + ".o")
            subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                                  "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
            objs.append(str(obj))
        cmd += ["-DWITH_ENGINE", f"-I{core}"]
        print(f"board engine: {core}")
    elif strict:
        print(f"FAIL: the board firmware is missing ({core}; ULTRA_REQUIRE_DEPS=1)")
        return 1
    else:
        print(f"board firmware not found ({core}): the engine and chain tests skipped")
    cmd += [str(HERE / "ultra_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
