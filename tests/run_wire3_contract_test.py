"""Production protocol 0.3 contract; the board descriptors fixture against the receiver firmware's generated ones."""
import os
from pathlib import Path
import re
import subprocess
import sys

from receiver_dependency import firmware_dir, receiver_contract

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build" / "wire3"

def firmware_descriptors(fw: Path):
    """the board descriptors as the firmware generates them (src/board/*/descriptor_gen.h): V203 and V407 Digital-M,
    Analog-M, in tests/fixtures/board_descriptors.h's order"""
    def arrays(path):
        s = path.read_text(encoding="utf-8")
        out = []
        for part in (re.split(r"^#else\s*$", s, flags=re.M) if "#else" in s else [s]):
            m = re.search(r"board_descriptor\[BOARD_DESCRIPTOR_BYTES\] = \{(.*?)\};", part, re.S)
            out.append(bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", m.group(1))))
        return out
    v407, v203 = arrays(fw / "src/board/ur_digital_m/descriptor_gen.h")
    return [v203, v407, *arrays(fw / "src/board/ur_analog_m/descriptor_gen.h")]


def fixture_descriptors():
    s = (HERE / "fixtures" / "board_descriptors.h").read_text(encoding="utf-8")
    return [bytes(int(x, 16) for x in re.findall(r"0x([0-9a-f]{2})", m))
            for m in re.findall(r"DESC_\w+\[\d+\] = \{(.*?)\};", s, re.S)]


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    fw = firmware_dir()
    receiver_contract(fw)
    if fixture_descriptors() != firmware_descriptors(fw):   # (the fixture is the firmware's)
        print(f"tests/fixtures/board_descriptors.h differs from {fw}'s src/board/*/descriptor_gen.h")
        return 1
    print(f"board descriptors fixture = {fw}'s descriptor_gen.h (3 descriptors)")
    for name in ("wire3_contract_test", "wire3_rate_test"):
      exe = BUILD / (name + (".exe" if sys.platform == "win32" else ""))
      subprocess.run([sys.executable,"-m","ziglang","c++","-std=c++17","-O1","-Wall","-Wextra","-Werror",
                    "-Wno-nullability-completeness","-fsanitize=undefined","-fno-sanitize-recover=all",
                    str(HERE / (name + ".cpp")),"-o",str(exe)],check=True)
      subprocess.run([str(exe)],check=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())
