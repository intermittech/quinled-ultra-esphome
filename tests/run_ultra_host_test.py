"""Build and run tests/ultra_host_test.cpp with the zig C/C++ compiler (pip install ziglang).

The board's light engine (the receiver firmware, receiver_dependency.py) is compiled in and the chain test runs on it;
the protocol header copy must equal the firmware's and the protocol checkout's at its HEAD (ci.yml PROTOCOL_REF), and
the test's vectors the protocol's vectors.json. Both checkouts are required.
"""

import json
import os
from receiver_dependency import firmware_dir, protocol_dir, receiver_contract, require
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_host_test.exe" if sys.platform == "win32" else "ultra_host_test")
    core = receiver_contract(firmware_dir())
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                 "-fsanitize=undefined", "-fno-sanitize-recover=all"]
    objs = []
    copy = (ROOT / "components" / "ultra_line" / "ultra_protocol.h").read_bytes().replace(b"\r\n", b"\n")
    protocol = protocol_dir()
    proto = require(protocol / "include" / "ultra_protocol.h", "the protocol checkout")
    # the header as committed at the checkout's HEAD (CI: PROTOCOL_REF; tests/check_pins.py checks the commit)
    git = ["git", "-C", str(protocol)]
    head = subprocess.run(git + ["show", "HEAD:include/ultra_protocol.h"], capture_output=True)
    if head.returncode != 0:
        print(f"FAIL: {protocol} is not a git checkout (git show HEAD:include/ultra_protocol.h)")
        return 1
    tag = subprocess.run(git + ["describe", "--tags", "--always"], capture_output=True, text=True).stdout.strip()
    if head.stdout.replace(b"\r\n", b"\n") != copy:
        print(f"FAIL: components/ultra_line/ultra_protocol.h differs from {proto} ({tag})")
        return 1
    print(f"ultra_protocol.h matches {proto} ({tag})")
    # the test's copy of protocol test vectors (VEC_<NAME> = "<hex>") against the protocol repository's vectors.json
    vectors = require(protocol / "test-vectors" / "vectors.json", "the protocol checkout")
    known = {v["name"]: v["hex"] for v in json.loads(vectors.read_text(encoding="utf-8"))["vectors"]}
    text = "\n".join((HERE / name).read_text(encoding="utf-8") for name in ("ultra_host_test.cpp", "wire3_rate_test.cpp"))
    for name, hexstr in re.findall(r'VEC_([A-Z0-9_]+) =\s*"([0-9a-f]+)"', text):
        if known.get(name.lower()) != hexstr:
            print(f"FAIL: test vector {name.lower()} differs from {vectors}")
            return 1
        print(f"test vector {name.lower()} matches {vectors.name}")
    for src in ("ultra.c", "ultra_tables.c", "math32.c", "config.c", "presets.c"):
        obj = BUILD / (src[:-2] + ".o")
        subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-fsanitize=undefined",
                              "-fno-sanitize-recover=all", "-c", str(core / src), "-o", str(obj)], check=True)
        objs.append(str(obj))
    cmd += ["-DWITH_ENGINE", f"-I{core}"]
    print(f"board engine: {core}")
    cmd += [str(HERE / "ultra_host_test.cpp"), *objs, "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
