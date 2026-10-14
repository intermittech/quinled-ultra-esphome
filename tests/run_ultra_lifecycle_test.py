"""Build and run tests/ultra_lifecycle_test.cpp with the zig C/C++ compiler (pip install ziglang).

The test runs methods of the component that need ESPHome / ESP-IDF (the light's setup, the update entity's install and
downloads, the hub's loop and transmit) with stand-ins for those. This script first cuts their bodies unchanged from
components/dig2analog_ultra into tests/build/lifecycle/*.inc, so the test always runs the current code.
"""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
COMP = ROOT / "components" / "dig2analog_ultra"
BUILD = HERE / "build" / "lifecycle"
ZIG = [sys.executable, "-m", "ziglang"]

# <name>.inc: the definitions in a source file that start with these signatures
PIECES = {
    "is_lit.inc": ("ultra_light.h", ["bool is_lit("]),
    "light_methods.inc": (
        "ultra_light.cpp",
        ["static uint8_t scene_fade(", "static uint16_t mired10(", "void UltraLight::setup_state(",
         "void UltraLight::send_opt_(", "void UltraLight::transition_done_(", "void UltraLight::lit_("],
    ),
    "update_methods.inc": (
        "fw_update.cpp",
        ["bool UltraFirmwareUpdate::fetch_(", "void UltraFirmwareUpdate::start_install_(",
         "bool UltraFirmwareUpdate::take_shared_image_("],
    ),
    "hub_loop.inc": ("dig2analog_ultra.cpp", ["void UltraHub::loop("]),
    "hub_transmit.inc": ("dig2analog_ultra.cpp", ["void UltraHub::transmit_("]),
}


def definition(text: str, signature: str) -> str:
    """The definition that starts with signature, through its closing brace (braces in comments / strings would
    confuse it: the methods cut here have none)."""
    start = text.index(signature)
    i, depth = text.index("{", start), 0
    while True:
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        i += 1
        if depth == 0:
            return text[start:i]


def main() -> int:
    BUILD.mkdir(parents=True, exist_ok=True)
    for name, (src, signatures) in PIECES.items():
        text = (COMP / src).read_text(encoding="utf-8")
        (BUILD / name).write_text("\n\n".join(definition(text, s) for s in signatures) + "\n", encoding="utf-8")
    # loop()'s QUEUED case (an install's turn), up to the next case
    loop = definition((COMP / "fw_update.cpp").read_text(encoding="utf-8"), "void UltraFirmwareUpdate::loop(")
    (BUILD / "update_queued.inc").write_text(loop[loop.index("    case QUEUED:"):loop.index("    case DOWNLOADING:")],
                                             encoding="utf-8")
    exe = BUILD / ("ultra_lifecycle_test.exe" if sys.platform == "win32" else "ultra_lifecycle_test")
    cmd = ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wno-unused-variable", "-Wno-unused-function",
                 "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-I{BUILD}",
                 str(HERE / "ultra_lifecycle_test.cpp"), "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return subprocess.run([str(exe)]).returncode


if __name__ == "__main__":
    sys.exit(main())
