"""Pin consistency: this repository, the protocol and the receiver firmware name the same commits.

  - ci.yml PROTOCOL_REF = the protocol checkout's HEAD = the firmware's own protocol pin (test/test_protocol_copy.py
    PROTOCOL_REF at FIRMWARE_REF);
  - ci.yml FIRMWARE_REF = the firmware checkout's HEAD;
  - components/ultra_line/ultra_protocol.h = the protocol's include/ultra_protocol.h at PROTOCOL_REF = the firmware's
    src/core/ultra_protocol.h at FIRMWARE_REF;
  - every vendored sender file (run_port_job_test.py VENDORED) = the protocol's sender/src at PROTOCOL_REF.

A pin that moves alone fails here with what to move with it. The checkouts: receiver_dependency.py (both required).
Usage: python tests/check_pins.py
"""
from pathlib import Path
import re
import subprocess
import sys

from receiver_dependency import firmware_dir, protocol_dir, require
from run_port_job_test import VENDORED

ROOT = Path(__file__).resolve().parent.parent
COMP = ROOT / "components" / "ultra_line"
CI = ROOT / ".github" / "workflows" / "ci.yml"


def lf(b: bytes) -> bytes:
    return b.replace(b"\r\n", b"\n")


def git(repo: Path, *args) -> subprocess.CompletedProcess:
    return subprocess.run(["git", "-C", str(repo), *args], capture_output=True, timeout=60)


def pin(name: str, text: str) -> str:
    m = re.search(rf"^\s*{name}:\s*([0-9a-f]{{40}})\b", text, re.M)
    if not m:
        raise SystemExit(f"FAIL: ci.yml has no {name}: <40-digit commit>")
    return m.group(1)


def main() -> int:
    ci = CI.read_text(encoding="utf-8")
    proto_ref, fw_ref = pin("PROTOCOL_REF", ci), pin("FIRMWARE_REF", ci)
    protocol = require(protocol_dir(), "the protocol checkout")
    firmware = require(firmware_dir(), "the receiver firmware")
    bad = []

    def head_is(repo: Path, ref: str, what: str, env: str) -> bool:
        head = git(repo, "rev-parse", "HEAD")
        if head.returncode != 0:
            bad.append(f"{what} ({repo}) is not a git checkout: {env} must name one at {ref[:12]}")
            return False
        if head.stdout.decode().strip() != ref:
            bad.append(f"{what} ({repo}) is at {head.stdout.decode().strip()[:12]}, ci.yml pins {ref[:12]}: check out "
                       f"the pin, or move the pin (and everything below that names it)")
            return False
        return True

    def show(repo: Path, ref: str, path: str) -> bytes | None:
        r = git(repo, "show", f"{ref}:{path}")
        if r.returncode != 0:
            bad.append(f"{path} is not in {repo} at {ref[:12]}")
            return None
        return lf(r.stdout)

    head_is(protocol, proto_ref, "the protocol checkout", "ULTRA_PROTOCOL")
    head_is(firmware, fw_ref, "the firmware checkout", "ULTRA_FIRMWARE")

    # the firmware's own protocol pin
    copy_test = show(firmware, fw_ref, "test/test_protocol_copy.py")
    if copy_test is not None:
        m = re.search(rb"^PROTOCOL_REF = '([0-9a-f]{40})'", copy_test, re.M)
        if not m:
            bad.append("the firmware's test/test_protocol_copy.py has no PROTOCOL_REF")
        elif m.group(1).decode() != proto_ref:
            bad.append(f"the firmware at {fw_ref[:12]} pins protocol {m.group(1).decode()[:12]}, ci.yml PROTOCOL_REF is "
                       f"{proto_ref[:12]}: move FIRMWARE_REF to a firmware commit on the same protocol (or PROTOCOL_REF)")

    # one protocol header, three copies
    mine = lf((COMP / "ultra_protocol.h").read_bytes())
    for repo, ref, path, what in ((protocol, proto_ref, "include/ultra_protocol.h", "the protocol"),
                                  (firmware, fw_ref, "src/core/ultra_protocol.h", "the firmware")):
        theirs = show(repo, ref, path)
        if theirs is not None and theirs != mine:
            bad.append(f"components/ultra_line/ultra_protocol.h differs from {what}'s {path} at {ref[:12]}")

    # the vendored sender
    for name in VENDORED:
        theirs = show(protocol, proto_ref, f"sender/src/{name}")
        if theirs is not None and theirs != lf((COMP / name).read_bytes()):
            bad.append(f"components/ultra_line/{name} differs from the protocol's sender/src/{name} at {proto_ref[:12]}")

    for b in bad:
        print("FAIL " + b)
    print(f"pins: {'FAIL' if bad else 'ok'}: protocol {proto_ref[:12]}, firmware {fw_ref[:12]} "
          f"(its protocol pin, the header in 3 copies, {len(VENDORED)} vendored sender files)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
