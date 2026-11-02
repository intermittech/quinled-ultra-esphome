"""Build and run tests/port_job_test.cpp with the zig C/C++ compiler (pip install ziglang): the port job's glue
(port_job.h) against the sender library's own port job, compiled from the component's vendored sender sources.

First the copy check: every vendored file (VENDORED) is byte-identical to sender/src of the protocol repository at the
pin (ci.yml PROTOCOL_REF), in the protocol checkout (receiver_dependency.py; required).
ULTRA_SENDER_SRC=<sender/src> builds against another copy instead. Built with ULTRA_SENDER_MAX_BOARDS=50 (a
Dig2Analog-Plus-5ch pin's boards, as update.py builds the component).

The threaded handover test (tests/port_job_threads.inc) runs the job's task and the main loop as two threads on the
component's own methods: they are cut unchanged from components/ultra_line into tests/build/port_job/*.inc first
(THREAD_PIECES), so the test always runs the current code.
"""

import os
from pathlib import Path
import re
import subprocess
import sys

from receiver_dependency import protocol_dir

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build" / "port_job"
ZIG = [sys.executable, "-m", "ziglang"]
SENDER_C = ("ultra_sender.c", "ultra_sender_bits.c", "ultra_sender_descriptor.c", "ultra_sender_profile.c",
            "ultra_sender_rc.c", "ultra_sender_update.c", "ultra_sender_job.c")
# every file of the protocol's sender/src the component carries, byte-identical
VENDORED = SENDER_C + tuple(n[:-2] + ".h" for n in SENDER_C) + (
    "ultra_sender_internal.h", "ultra_protocol.h", "ultra_update.h", "ultra_transport.h", "ultra_dport.h")
COMP = ROOT / "components" / "ultra_line"
# <name>.inc: the definitions in a source file that start with these signatures (a struct: with its ;)
THREAD_PIECES = {
    "thread_job_run.inc": ("fw_update.cpp", ["struct UltraFirmwareUpdate::JobRun final"]),
    "thread_update.inc": (
        "fw_update.cpp",
        ["void UltraFirmwareUpdate::job_task_(", "void UltraFirmwareUpdate::service_job_(",
         "void UltraFirmwareUpdate::abort("],
    ),
    "thread_hub.inc": (
        "ultra_line.cpp",
        ["bool UltraHub::boards_powered_(", "bool UltraHub::receivers_ready_(", "int64_t UltraHub::job_frame_start_us(",
         "bool UltraHub::job_send(", "bool UltraHub::lease_line("],
    ),
}


def definition(text: str, signature: str) -> str:
    """The definition that starts with signature, through its closing brace (braces in comments / strings would
    confuse it: the definitions cut here have none)."""
    start = text.index(signature)
    i, depth = text.index("{", start), 0
    while True:
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        i += 1
        if depth == 0:
            return text[start:i] + (";" if signature.startswith("struct ") else "")


def sender_dir():
    env = os.environ.get("ULTRA_SENDER_SRC")
    if env and (Path(env) / "ultra_sender_job.h").exists():
        return Path(env)
    return COMP if (COMP / "ultra_sender_job.h").exists() else None


def copy_check() -> bool:
    """The vendored files against sender/src at the pin; False: a difference or no protocol checkout."""
    protocol = protocol_dir()
    ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
    ref = re.search(r"PROTOCOL_REF:\s*([0-9a-f]{7,40})", ci).group(1)
    if not (protocol / ".git").exists():
        print(f"FAIL: the vendored sender copy check needs the protocol checkout ({protocol}; ULTRA_PROTOCOL)")
        return False
    ok = True
    for name in VENDORED:
        pinned = subprocess.run(["git", "-C", str(protocol), "show", f"{ref}:sender/src/{name}"], capture_output=True)
        if pinned.returncode != 0:
            print(f"FAIL: sender/src/{name} is not in the protocol at {ref[:12]}")
            ok = False
        elif pinned.stdout.replace(b"\r\n", b"\n") != (COMP / name).read_bytes().replace(b"\r\n", b"\n"):
            print(f"FAIL: components/ultra_line/{name} differs from sender/src/{name} at {ref[:12]}")
            ok = False
    if ok:
        print(f"vendored sender sources: {len(VENDORED)} files identical to sender/src at {ref[:12]}")
    return ok


def main() -> int:
    src = sender_dir()
    if src is None:
        print("FAIL: no sender sources (components/ultra_line/ultra_sender_job.h)")
        return 1
    if src == COMP and not copy_check():
        return 1
    BUILD.mkdir(parents=True, exist_ok=True)
    for name, (source, signatures) in THREAD_PIECES.items():
        text = (COMP / source).read_text(encoding="utf-8")
        (BUILD / name).write_text("\n\n".join(definition(text, s) for s in signatures) + "\n", encoding="utf-8")
    defines = ["-DULTRA_SENDER_MAX_BOARDS=50"]
    objs = []
    for name in SENDER_C:
        obj = BUILD / (name[:-2] + ".o")
        subprocess.run(ZIG + ["cc", "-std=c99", "-O1", "-Wall", "-Werror", "-fsanitize=undefined",
                              "-fno-sanitize-recover=all", *defines, f"-I{src}", "-c", str(src / name), "-o", str(obj)],
                       check=True)
        objs.append(str(obj))
    exe = BUILD / ("port_job_test.exe" if sys.platform == "win32" else "port_job_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", *defines, f"-I{src}",
                          str(HERE / "port_job_test.cpp"), *objs, "-o", str(exe)]
                   + ([] if sys.platform == "win32" else ["-pthread"]), check=True)   # (the threaded handover test)
    print(f"sender sources: {src}")
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
