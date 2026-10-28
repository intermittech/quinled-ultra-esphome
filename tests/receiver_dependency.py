"""The two repositories the host tests run against, both required: the protocol (quinled-ultra-protocol at ci.yml's
PROTOCOL_REF) and the receiver firmware (quinled-ultra-firmware at ci.yml's FIRMWARE_REF).

Where they are: ULTRA_PROTOCOL=<checkout> / ULTRA_FIRMWARE=<checkout>, else checkouts next to this repository. A missing
one is a failure, never a skip; a firmware whose protocol header differs from the component's is never used.
tests/check_pins.py checks that the checkouts are at the pinned commits.
"""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent


def protocol_dir() -> Path:
    return Path(os.environ.get("ULTRA_PROTOCOL", ROOT.parent / "quinled-ultra-protocol"))


def firmware_dir() -> Path:
    return Path(os.environ.get("ULTRA_FIRMWARE", ROOT.parent / "quinled-ultra-firmware"))


def require(path: Path, what: str) -> Path:
    """path, or a failure naming what is missing and how to point at it"""
    if not path.exists():
        raise SystemExit(f"FAIL: {what} is missing ({path}): check it out next to this repository or set "
                         f"{'ULTRA_FIRMWARE' if 'firmware' in what else 'ULTRA_PROTOCOL'}")
    return path


def receiver_contract(firmware: Path) -> Path:
    """the firmware's src/core, after checking that its protocol header is the component's, byte for byte"""
    core = firmware / "src" / "core"
    header = require(core / "ultra_protocol.h", "the receiver firmware")
    production = ROOT / "components/ultra_line/ultra_protocol.h"
    if header.read_bytes().replace(b"\r\n", b"\n") != production.read_bytes().replace(b"\r\n", b"\n"):
        raise SystemExit(f"FAIL: {header} differs from components/ultra_line/ultra_protocol.h: the receiver contracts "
                         "need the firmware of the same protocol commit (ci.yml FIRMWARE_REF, tests/check_pins.py)")
    return core
