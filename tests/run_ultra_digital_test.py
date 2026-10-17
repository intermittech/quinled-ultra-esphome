"""Build and run tests/ultra_digital_test.cpp with the zig C/C++ compiler (pip install ziglang): the hub's side of the
Ultra-Digital receiver's local pixel effects (PIXEL_FX, CONFIG 129-139, page 0x81), and the checks that tie the files
together:
  - the chips of components/ultra_digital/__init__.py match the C++ table (ultra_pixel_fx.h);
  - the catalogue copy (components/ultra_digital/pixel_fx_catalogue.json) is byte-identical to the protocol repository's
    (catalogue/), and passes its checker;
  - the golden PIXEL_FX slot of the test is the protocol repository's test vector slot_pixel_fx (when that checkout is next
    to this one).
The light platform itself is covered by tests/validate_configs.py (its schema) and the example compiles.
"""

from pathlib import Path
import json
import os
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]
PROTOCOL = ROOT.parent / "quinled-ultra-protocol"


# the light's own methods, cut unchanged from components/ultra_digital/ultra_digital.cpp for tests/ultra_digital_light_test.cpp
PIECES = ["PixelLight *PixelLight::find(", "void PixelLight::transition_requested(", "PixelLight *PixelFxEffect::light_(",
          "void PixelFxEffect::start(", "void PixelFxEffect::stop(", "void PixelLight::effect_started(",
          "void PixelLight::effect_stopped(", "void PixelLight::setup() {", "void PixelLight::on_light_remote_values_update(",
          "void ExclusiveWatcher::on_light_remote_values_update(", "void PixelLight::other_changed_(",
          "void PixelLight::queue_fx_(", "void PixelLight::release_(", "void PixelLight::write_state(", "uint8_t PixelLight::shown_effect_(",
          "void PixelLight::loop(", "void PixelLight::send_(", "void PixelLight::set_param(",
          "uint8_t PixelLight::get_param(", "void PixelLight::set_palette_index(", "void PixelLight::set_color2(",
          "void PixelLight::set_color3(", "void PixelLight::pixel_page_("]


def definition(text: str, signature: str) -> str:
    """The definition that starts with signature, through its closing brace (the methods cut here have no braces in
    comments or strings)"""
    start = text.index(signature)
    i, depth = text.index("{", start), 0
    while True:
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        i += 1
        if depth == 0:
            return text[start:i]


def light_test() -> int:
    out = BUILD / "digital"
    out.mkdir(parents=True, exist_ok=True)
    text = (ROOT / "components" / "ultra_digital" / "ultra_digital.cpp").read_text(encoding="utf-8")
    body = "\n\n".join(definition(text, s) for s in PIECES) + "\n"
    (out / "light_methods.inc").write_text(body, encoding="utf-8")
    exe = out / ("ultra_digital_light_test.exe" if sys.platform == "win32" else "ultra_digital_light_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-function",
                          "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-I{out}",
                          str(HERE / "ultra_digital_light_test.cpp"), "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    return run.returncode


# the digital port's methods, cut unchanged from components/dig2analog_ultra/dig2analog_ultra.cpp for
# tests/ultra_digital_transport_test.cpp
TRANSPORT_ENCODER = ["static size_t IRAM_ATTR HOT dp_encoder_callback("]
TRANSPORT_METHODS = ["bool UltraHub::digital_setup_(", "void UltraHub::digital_pixels(", "void UltraHub::digital_note_fx(",
                     "void UltraHub::digital_retry(", "void UltraHub::digital_save_(", "void UltraHub::digital_window_end_(",
                     "void UltraHub::loop_digital_(", "void UltraHub::transmit_digital_("]


def planner_copy() -> bool:
    """components/dig2analog_ultra/ultra_dport.h is the sender library's, byte for byte (the planner is tested there,
    against the receiver's own delay line)"""
    mine = ROOT / "components" / "dig2analog_ultra" / "ultra_dport.h"
    theirs = PROTOCOL / "sender" / "src" / "ultra_dport.h"
    if not theirs.exists():
        if os.environ.get("ULTRA_REQUIRE_DEPS") == "1":
            print(f"FAIL: the protocol repository is missing ({theirs}; ULTRA_REQUIRE_DEPS=1)")
            return False
        print(f"quinled-ultra-protocol not found ({theirs}): the planner copy check skipped")
        return True
    # the working tree's file (the sender library's CI tag is the same file once released)
    same = mine.read_bytes().replace(b"\r\n", b"\n") == theirs.read_bytes().replace(b"\r\n", b"\n")
    print(f"{'ok  ' if same else 'FAIL'} ultra_dport.h is the sender library's copy")
    return same


def transport_test() -> int:
    out = BUILD / "transport"
    out.mkdir(parents=True, exist_ok=True)
    text = (ROOT / "components" / "dig2analog_ultra" / "dig2analog_ultra.cpp").read_text(encoding="utf-8")
    (out / "dp_encoder.inc").write_text("\n\n".join(definition(text, s) for s in TRANSPORT_ENCODER) + "\n", encoding="utf-8")
    (out / "dp_methods.inc").write_text("\n\n".join(definition(text, s) for s in TRANSPORT_METHODS) + "\n", encoding="utf-8")
    exe = out / ("ultra_digital_transport_test.exe" if sys.platform == "win32" else "ultra_digital_transport_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-function",
                          "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                          str(HERE / "ultra_digital_transport_test.cpp"), "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    return run.returncode


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ultra_digital_test.exe" if sys.platform == "win32" else "ultra_digital_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", str(HERE / "ultra_digital_test.cpp"),
                          "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write("".join(l + "\n" for l in run.stdout.splitlines() if not l.startswith(("CHIP ", "VEC_"))))
    sys.stderr.write(run.stderr)
    bad = run.returncode != 0
    bad |= light_test() != 0
    bad |= not planner_copy()
    bad |= transport_test() != 0
    # what the hub put on the wire, on the receiver's own delay line (the firmware's model; skipped without the firmware)
    bad |= subprocess.run([sys.executable, str(HERE / "run_ultra_dline_replay_test.py")]).returncode != 0

    # 1. the chips: the YAML's table against the header's
    sys.path.insert(0, str(ROOT / "components"))
    from ultra_digital import CHIPS   # noqa: E402  (imports esphome: run with ESPHome's Python)

    cpp = [l.split() for l in run.stdout.splitlines() if l.startswith("CHIP ")]
    macros = dict(re.findall(r"#define (ULTRA_PXC_\w+)\s+(\d+)u", (ROOT / "components/dig2analog_ultra/ultra_protocol.h").read_text()))
    for name, (idx, nbytes, own, macro) in CHIPS.items():
        row = next((r for r in cpp if int(r[1]) == idx), None)
        good = row is not None and int(row[2]) == nbytes and row[3] == own and row[4] == macros.get(macro)
        print(f"{'ok  ' if good else 'FAIL'} chip {name}: index {idx}, {nbytes} bytes, {own}, {macro}")
        bad |= not good
    if len(cpp) != len(CHIPS):
        print(f"FAIL chips: {len(CHIPS)} in light.py, {len(cpp)} in ultra_pixel_fx.h")
        bad = True

    # 2. the catalogue: identical to the protocol repository's, and valid
    mine = ROOT / "components/ultra_digital/pixel_fx_catalogue.json"
    theirs = PROTOCOL / "catalogue/pixel_fx_catalogue.json"
    if theirs.exists():
        same = mine.read_bytes().replace(b"\r\n", b"\n") == theirs.read_bytes().replace(b"\r\n", b"\n")
        print(f"{'ok  ' if same else 'FAIL'} the catalogue is the protocol repository's copy")
        bad |= not same
        checker = PROTOCOL / "catalogue/check_catalogue.py"
        proc = subprocess.run([sys.executable, str(checker), str(mine)], capture_output=True, text=True)
        print(("ok   " if proc.returncode == 0 else "FAIL ") + proc.stdout.strip())
        bad |= proc.returncode != 0
    elif os.environ.get("ULTRA_REQUIRE_DEPS") == "1":
        print(f"FAIL: the protocol repository is missing ({PROTOCOL}; ULTRA_REQUIRE_DEPS=1)")
        bad = True
    else:
        print(f"quinled-ultra-protocol not found ({PROTOCOL}): the catalogue copy check skipped")
    from ultra_digital import catalogue   # noqa: E402

    cat = catalogue.load()
    print(f"ok   catalogue {cat.version}: {len(cat.effects)} effects, {len(cat.palettes)} palettes, Solid = {cat.solid_id}")

    # 3. the golden slot is the protocol repository's test vector
    vec = next((l for l in run.stdout.splitlines() if l.startswith("VEC_SLOT_PIXEL_FX")), None)
    vectors = PROTOCOL / "test-vectors/vectors.json"
    if vec and vectors.exists():
        hexstr = re.search(r'"([0-9a-f]+)"', vec).group(1)
        known = {v["name"]: v["hex"] for v in json.loads(vectors.read_text(encoding="utf-8"))["vectors"]}
        good = known.get("slot_pixel_fx") == hexstr
        print(f"{'ok  ' if good else 'FAIL'} the golden slot is the protocol test vector slot_pixel_fx")
        bad |= not good
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
