"""Build and run tests/ur_digital_m_test.cpp with the zig C/C++ compiler (pip install ziglang): the hub's side of the
Diff-Power Ultra Receiver Digital-Medium's local pixel effects (PIXEL_FX, CONFIG 129-139, page 0x11), and the checks that tie the files
together:
  - the chips of components/ur_digital_m/__init__.py match the C++ table (ultra_pixel_fx.h);
  - its pixel limits (MAX_PIXELS) are the boards' own: the firmware descriptors' pixel records (tests/fixtures/);
  - the catalogue copy (components/ur_digital_m/pixel_fx_catalogue.json) matches the protocol repository's
    (catalogue/) apart from HA group metadata, and passes its checker;
  - the golden PIXEL_FX slot of the test is the protocol repository's test vector slot_pixel_fx (when that checkout is next
    to this one).
The light and colour entity methods run in ur_digital_m_light_test.cpp with ESPHome stand-ins; validate_configs.py
checks their schema/codegen, and the examples cover ESP32 compilation. ULTRA_PROTOCOL overrides the sibling protocol
checkout path for nested worktrees.
"""

from pathlib import Path
import json
import os
import re
import subprocess
import sys

from receiver_dependency import protocol_dir

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / "build"
ZIG = [sys.executable, "-m", "ziglang"]
PROTOCOL = protocol_dir()


# the light's own methods, cut unchanged from components/ur_digital_m/ur_digital_m.cpp for tests/ur_digital_m_light_test.cpp
PIECES = ["PixelLight *PixelLight::find(", "void PixelLight::transition_requested(", "PixelLight *PixelFxEffect::light_(",
          "void PixelFxEffect::start(", "void PixelFxEffect::stop(", "void PixelLight::effect_started(",
          "void PixelLight::effect_stopped(", "void PixelLight::setup() {", "void PixelLight::on_light_remote_values_update(",
          "void ExclusiveWatcher::on_light_remote_values_update(", "void PixelLight::other_changed_(",
          "void PixelLight::queue_fx_(", "void PixelLight::release_(", "void PixelLight::write_state(", "uint8_t PixelLight::shown_effect_(",
          "void PixelLight::loop(", "void PixelLight::send_(", "void PixelLight::set_param(",
          "uint8_t PixelLight::get_param(", "void PixelLight::set_palette_index(", "void PixelLight::set_color2(",
          "void PixelLight::set_color3(", "void PixelLight::pixel_page_(", "void PixelLight::send_line_mode_(",
          "void PixelLight::line_identity_(", "std::string PixelLight::line_text_(", "std::string PixelLight::status_text_("]
COLOUR_PIECES = ["bool PixelColour::parse_color(", "std::string PixelColour::format_color(",
                 "void PixelColour::setup(", "void PixelColour::control(", "void PixelColour::publish_color("]


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


def descriptor_pixel_limits() -> dict:
    """tests/fixtures/board_descriptors.h: each descriptor's pixel record (TLV type 4) as {bytes per pixel: max_pixels}"""
    s = (HERE / "fixtures" / "board_descriptors.h").read_text(encoding="utf-8")
    arrays = {name: bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", body))
              for name, body in re.findall(r"DESC_(\w+)\[\d+\] = \{(.*?)\};", s, re.S)}
    names = dict(re.findall(r'\{"(\w+)", DESC_(\w+),', s))
    out = {}
    for name, key in names.items():
        b, p = arrays[key], 11
        while p + 2 <= len(b):
            kind, size, v = b[p], b[p + 1], b[p + 2:p + 2 + b[p + 1]]
            if kind == 4 and size >= 31 and v[0] == 0:
                out[name] = {3 + i: v[6 + 2 * i] << 8 | v[7 + 2 * i] for i in range(3)}
            p += 2 + size
    return out


def light_test() -> int:
    out = BUILD / "digital"
    out.mkdir(parents=True, exist_ok=True)
    text = (ROOT / "components" / "ur_digital_m" / "ur_digital_m.cpp").read_text(encoding="utf-8")
    body = "\n\n".join(definition(text, s) for s in PIECES + COLOUR_PIECES) + "\n"
    (out / "light_methods.inc").write_text(body, encoding="utf-8", newline="\n")
    header = (ROOT / "components/ur_digital_m/ur_digital_m.h").read_text(encoding="utf-8")
    for filename, signature in (("colour_class.inc", "class PixelColour final"),
                                ("params_action.inc", "template<typename... Ts> class EffectParamsAction")):
        (out / filename).write_text(definition(header, signature) + ";\n", encoding="utf-8", newline="\n")
    exe = out / ("ur_digital_m_light_test.exe" if sys.platform == "win32" else "ur_digital_m_light_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-function",
                          "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all", f"-I{out}",
                          str(HERE / "ur_digital_m_light_test.cpp"), "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    return run.returncode


# the digital port's methods, cut unchanged from components/ultra_line/ultra_line.cpp for
# tests/ur_digital_m_transport_test.cpp
TRANSPORT_ENCODER = ["static size_t IRAM_ATTR HOT dp_encoder_callback("]
TRANSPORT_METHODS = ["bool UltraHub::digital_setup_(", "void UltraHub::digital_pixels(", "void UltraHub::digital_note_fx(",
                     "void UltraHub::digital_retry(", "void UltraHub::digital_save_(", "void UltraHub::digital_window_end_(",
                     "void UltraHub::digital_window_track_(", "void UltraHub::loop_digital_(",
                     "void UltraHub::transmit_digital_(", "bool UltraHub::lease_line(", "bool UltraHub::job_send("]


def planner_copy() -> bool:
    """components/ultra_line/ultra_dport.h is the sender library's, byte for byte (the planner is tested there,
    against the receiver's own delay line)"""
    mine = ROOT / "components" / "ultra_line" / "ultra_dport.h"
    theirs = PROTOCOL / "sender" / "src" / "ultra_dport.h"
    if not theirs.exists():
        print(f"FAIL: the protocol repository is missing ({theirs}; ULTRA_PROTOCOL)")
        return False
    # the working tree's file (the sender library's CI tag is the same file once released)
    same = mine.read_bytes().replace(b"\r\n", b"\n") == theirs.read_bytes().replace(b"\r\n", b"\n")
    print(f"{'ok  ' if same else 'FAIL'} ultra_dport.h is the sender library's copy")
    return same


def transport_test() -> int:
    out = BUILD / "transport"
    out.mkdir(parents=True, exist_ok=True)
    text = (ROOT / "components" / "ultra_line" / "ultra_line.cpp").read_text(encoding="utf-8")
    (out / "dp_encoder.inc").write_text("\n\n".join(definition(text, s) for s in TRANSPORT_ENCODER) + "\n", encoding="utf-8")
    (out / "dp_methods.inc").write_text("\n\n".join(definition(text, s) for s in TRANSPORT_METHODS) + "\n", encoding="utf-8")
    exe = out / ("ur_digital_m_transport_test.exe" if sys.platform == "win32" else "ur_digital_m_transport_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-unused-function",
                          "-Wno-nullability-completeness", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                          str(HERE / "ur_digital_m_transport_test.cpp"), "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    return run.returncode


def calibration_test() -> int:
    out = BUILD / "calibration"
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "components/ultra_line/ultra_line.cpp").read_text(encoding="utf-8")
    methods = ["void UltraHub::calibrate_boards(", "void UltraHub::cancel_calibration(",
               "bool UltraHub::any_lit_(", "void UltraHub::service_calibration_(", "void UltraHub::calibrate_receiver("]
    (out / "calibration_methods.inc").write_text("\n\n".join(definition(source, m) for m in methods), encoding="utf-8")
    exe = out / ("calibration.exe" if sys.platform == "win32" else "calibration")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                          "-Wno-unused-variable", "-Wno-nullability-completeness", "-fsanitize=undefined",
                          "-fno-sanitize-recover=all", f"-I{out}",
                          str(HERE / "ultra_pixel_calibration_hub_test.cpp"), "-o", str(exe)], check=True)
    return subprocess.run([str(exe)]).returncode


def main() -> int:
    if calibration_test() != 0:
        return 1
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / ("ur_digital_m_test.exe" if sys.platform == "win32" else "ur_digital_m_test")
    subprocess.run(ZIG + ["c++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-nullability-completeness",
                          "-fsanitize=undefined", "-fno-sanitize-recover=all", str(HERE / "ur_digital_m_test.cpp"),
                          "-o", str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    sys.stdout.write("".join(l + "\n" for l in run.stdout.splitlines() if not l.startswith(("CHIP ", "VEC_"))))
    sys.stderr.write(run.stderr)
    bad = run.returncode != 0
    bad |= light_test() != 0
    bad |= not planner_copy()
    bad |= transport_test() != 0
    # what the hub put on the wire, on the receiver's own delay line (the firmware's model)
    bad |= subprocess.run([sys.executable, str(HERE / "run_ultra_dline_replay_test.py")]).returncode != 0

    # 1. the chips: the YAML's table against the header's
    sys.path.insert(0, str(ROOT / "components"))
    from ur_digital_m import CHIPS   # noqa: E402  (imports esphome: run with ESPHome's Python)

    cpp = [l.split() for l in run.stdout.splitlines() if l.startswith("CHIP ")]
    macros = dict(re.findall(r"#define (ULTRA_PXC_\w+)\s+(\d+)u", (ROOT / "components/ultra_line/ultra_protocol.h").read_text()))
    for name, (idx, nbytes, own, macro) in CHIPS.items():
        row = next((r for r in cpp if int(r[1]) == idx), None)
        good = row is not None and int(row[2]) == nbytes and row[3] == own and row[4] == macros.get(macro)
        print(f"{'ok  ' if good else 'FAIL'} chip {name}: index {idx}, {nbytes} bytes, {own}, {macro}")
        bad |= not good
    if len(cpp) != len(CHIPS):
        print(f"FAIL chips: {len(CHIPS)} in light.py, {len(cpp)} in ultra_pixel_fx.h")
        bad = True

    # 2. the pixel limits: the YAML's table against the firmware descriptors' pixel records (max_pixels at 3 / 4 / 5 bytes)
    from ur_digital_m import MAX_PIXELS   # noqa: E402

    found = descriptor_pixel_limits()
    for board, limits in MAX_PIXELS.items():
        own = found.get(f"ur_digital_m_{board}")
        good = own == limits
        print(f"{'ok  ' if good else 'FAIL'} pixel limits {board}: {limits} (descriptor: {own})")
        bad |= not good
    if sorted(f"ur_digital_m_{b}" for b in MAX_PIXELS) != sorted(found):
        print(f"FAIL pixel limits: boards {sorted(MAX_PIXELS)} in __init__.py, {sorted(found)} in the fixture")
        bad = True
    from ur_digital_m.catalogue import BOARD_CATALOGUES   # noqa: E402
    if sorted(MAX_PIXELS) != sorted(BOARD_CATALOGUES):
        print(f"FAIL pixel limits: board_type {sorted(BOARD_CATALOGUES)}, MAX_PIXELS {sorted(MAX_PIXELS)}")
        bad = True

    # 3. the catalogue: identical protocol data apart from HA group metadata, and valid
    mine = ROOT / "components/ur_digital_m/pixel_fx_catalogue.json"
    theirs = PROTOCOL / "catalogue/pixel_fx_catalogue.json"
    if theirs.exists():
        from check_pixel_groups import without_ha_groups
        same = without_ha_groups(json.loads(mine.read_text(encoding="utf-8"))) == without_ha_groups(json.loads(theirs.read_text(encoding="utf-8")))
        print(f"{'ok  ' if same else 'FAIL'} the catalogue matches the protocol repository apart from HA groups")
        bad |= not same
        checker = PROTOCOL / "catalogue/check_catalogue.py"
        protocol_data = BUILD / "v203_protocol_catalogue.json"
        protocol_data.write_text(json.dumps(without_ha_groups(json.loads(mine.read_text(encoding="utf-8")))), encoding="utf-8")
        proc = subprocess.run([sys.executable, str(checker), str(protocol_data)], capture_output=True, text=True)
        print(("ok   " if proc.returncode == 0 else "FAIL ") + proc.stdout.strip())
        bad |= proc.returncode != 0
    else:
        print(f"FAIL: the protocol repository is missing ({PROTOCOL}; ULTRA_PROTOCOL)")
        bad = True
    from ur_digital_m import catalogue   # noqa: E402

    cat = catalogue.load()
    v407 = catalogue.load(board_type="v407")
    assert len(cat.effects) == 36 and len(cat.palettes) == 31
    assert len(v407.effects) == 114 and len(v407.palettes) == 49
    assert v407.effects[:36] == cat.effects and v407.palettes[:31] == cat.palettes
    assert [e["id"] for e in v407.effects] == list(range(1, 115))
    assert [p["id"] for p in v407.palettes] == list(range(49))
    assert all("design" in e for e in v407.effects[36:])
    assert all(len(p["stops"]) == 16 for p in v407.palettes[31:])
    resolved = [e for e in v407.effects if "default_palette_id" in e.get("design", {})]
    assert len(resolved) == 17
    palettes = {p["id"]: p["name"] for p in v407.palettes}
    assert all(palettes[e["design"]["default_palette_id"]] == e["design"]["default_palette_name"] for e in resolved)
    assert [p["name"] for p in v407.palettes[43:]] == ["Fig and Brass", "Mulberry Milk", "Clay and Linen", "Neon Noir", "Tide Pool", "Graphite Citrus"]
    assert all(len(catalogue.capabilities(board)) < 255 for board in catalogue.BOARD_CATALOGUES)
    from check_pixel_groups import check_groups
    check_groups()
    print(f"ok   catalogue {cat.version}: {len(cat.effects)} effects, {len(cat.palettes)} palettes, Solid = {cat.solid_id}")

    # Wire-3 target/scoped byte contracts are covered by run_wire3_contract_test.py.
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
