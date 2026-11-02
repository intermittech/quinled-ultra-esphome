"""Wire-3 processing LOW: every normal line has at least 500 us; codegen verifies RMT and PARLIO, and the line's
port type and descriptors."""

from pathlib import Path
import importlib.util
import subprocess
import sys

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("chain_reset", HERE.parent / "components" / "ultra_line" / "chain_reset.py")
cr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cr)

failures = 0


def check(cond, what):
    global failures
    print(("ok   " if cond else "FAIL ") + what)
    failures += not cond


class Id:
    """an ESPHome ID: compares and prints by its id"""

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return self.name


def hub(name="u", **extra):
    return {"id": Id(name), **extra}


def rx(hub_name, board, platform="ur_digital_m"):
    return {"platform": platform, "ultra_line_id": Id(hub_name), "board": board}


check(cr.CHAIN_FIRST_RESET_NS == 500000, "the floor is 500 us")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 1)]) == 500000, "a receiver alone on a normal line: the default 300 us becomes 500 us")
check(cr.reset_with_floor_ns(280001, hub(), [rx("u", 1)]) == 500000, "... and the shortest allowed reset too")
check(cr.reset_with_floor_ns(800000, hub(), [rx("u", 1)]) == 800000, "a longer reset stays")
check(cr.reset_with_floor_ns(500000, hub(), [rx("u", 1)]) == 500000, "exactly 500 us stays")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 3)]) == 500000, "a receiver behind other boards (board 3): the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 2)]) == 500000, "... board 2 too")
check(cr.reset_with_floor_ns(300000, hub(), [rx("other", 1)]) == 500000, "a receiver on another hub: the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 1, "dig2analog_plus")]) == 500000, "a dig2analog_plus light on board 1: the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(), []) == 500000, "no receiver: the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(), None) == 500000, "no lights at all: the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(digital={"reset_time": 300000}), [rx("u", 1)]) == 500000, "a digital port (the receiver is in a strip's data path): the same 500 us floor")
check(cr.reset_with_floor_ns(300000, hub(parlio_id=Id("bus")), [rx("u", 1)]) == 500000, "a PARLIO lane is a normal Ultra line: 500 us too")
check(cr.reset_with_floor_ns(300000, hub(parlio_id=Id("bus")), [rx("u", 2)]) == 500000, "... and board 2 of a lane stays")

CONFIG = """esphome:
  name: {name}
esp32:
  board: esp32dev
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [ultra_line, ur_digital_m]
logger:
ultra_line:
  - id: ultra
    pin: GPIO16
    num_boards: {boards}
{extra}light:
  - platform: ur_digital_m
    id: bar
    name: Bar
    ultra_line_id: ultra
    board: {board}
    chip: ws2812b
    num_leds: 90
"""


def codegen(name, boards, board, reset_us, expect_ns):
    work = HERE / "build" / "chain_reset" / name
    work.mkdir(parents=True, exist_ok=True)
    extra = f"    reset_time: {reset_us}us\n" if reset_us else ""
    (work / f"{name}.yaml").write_text(CONFIG.format(name=name, components=(HERE.parent / "components").as_posix(), boards=boards, board=board,
                                                     extra=extra), encoding="utf-8")
    run = subprocess.run([sys.executable, "-m", "esphome", "compile", f"{name}.yaml", "--only-generate"], cwd=work, capture_output=True, text=True,
                         timeout=600)
    main_cpp = next((work / ".esphome" / "build").glob("*/src/main.cpp"), None) if (work / ".esphome" / "build").exists() else None
    line = next((l for l in main_cpp.read_text(encoding="utf-8").splitlines() if "ultra->set_timing(" in l), "") if main_cpp else ""
    check(run.returncode == 0 and f", {expect_ns});" in line, f"{name}: the hub's set_timing ends with {expect_ns} ns ({line.strip() or run.stdout[-300:] + run.stderr[-300:]})")
    # the hub's port type (its board limit, start profile): a Diff-Power Ultra Receiver light (ur_digital_m) on the line
    # makes it a Diff-Power port, also without telemetry, a Diff-Power port or a digital port
    port = next((l for l in main_cpp.read_text(encoding="utf-8").splitlines() if "ultra->set_port_type(" in l), "") if main_cpp else ""
    check("ultra->set_port_type(1);" in port, f"{name}: the hub gets its line's port type, diff_power ({port.strip()})")


PARLIO_CONFIG = """esphome:
  name: parlio_alone
esp32:
  board: esp32-p4_r3
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [ultra_line, ultra_parlio, ur_digital_m]
logger:
ultra_parlio:
  - id: ports
    pins: [GPIO0]
ultra_line:
  - id: ultra
    parlio_id: ports
    lane: 0
    num_boards: 1
light:
  - platform: ur_digital_m
    id: bar
    name: Bar
    ultra_line_id: ultra
    board: 1
    chip: ws2812b
    num_leds: 90
"""


DESCRIPTOR_CONFIG = """esphome:
  name: pin_descriptors
esp32:
  board: esp32dev
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [ultra_line]
logger:
ultra_line:
  - id: ultra
    pin: GPIO16
    num_boards: 35
    line_rate:
      inventory_complete: true
      bootloader_profile_mask: 15
    every_board_descriptor: {file}
    descriptors:
      - board: 4
        file: {file}
"""


def codegen_descriptors():
    """A long Dig2Analog-Plus-5ch pin: one every_board_descriptor (one copy in the firmware, not one per board) and a
    board's own entry; the line is a d2a_pin."""
    work = HERE / "build" / "chain_reset" / "pin_descriptors"
    work.mkdir(parents=True, exist_ok=True)
    (work / "pin_descriptors.yaml").write_text(DESCRIPTOR_CONFIG.format(
        components=(HERE.parent / "components").as_posix(), file=(HERE / "fixtures" / "boot_info_v203.bin").as_posix()),
        encoding="utf-8")
    run = subprocess.run([sys.executable, "-m", "esphome", "compile", "pin_descriptors.yaml", "--only-generate"], cwd=work,
                         capture_output=True, text=True, timeout=600)
    main_cpp = next((work / ".esphome" / "build").glob("*/src/main.cpp"), None) if (work / ".esphome" / "build").exists() else None
    lines = main_cpp.read_text(encoding="utf-8").splitlines() if main_cpp else []
    every = [l for l in lines if "ultra->set_every_board_descriptor(" in l]
    own = [l for l in lines if "ultra->set_board_descriptor(" in l]
    port = [l for l in lines if "ultra->set_port_type(" in l]
    check(run.returncode == 0 and len(every) == 1 and len(own) == 1 and "set_board_descriptor(3, " in own[0],
          f"every_board_descriptor: one shared copy, board 4 its own (slot 3) ({len(every)} / {len(own)}; "
          f"{'' if run.returncode == 0 else run.stdout[-300:] + run.stderr[-300:]})")
    check(len(port) == 1 and "ultra->set_port_type(2);" in port[0], f"a pin without receivers is a d2a_pin ({port})")


def codegen_parlio():
    work = HERE / "build" / "chain_reset" / "parlio_alone"
    work.mkdir(parents=True, exist_ok=True)
    (work / "parlio_alone.yaml").write_text(PARLIO_CONFIG.format(components=(HERE.parent / "components").as_posix()), encoding="utf-8")
    run = subprocess.run([sys.executable, "-m", "esphome", "compile", "parlio_alone.yaml", "--only-generate"], cwd=work, capture_output=True, text=True,
                         timeout=600)
    main_cpp = next((work / ".esphome" / "build").glob("*/src/main.cpp"), None) if (work / ".esphome" / "build").exists() else None
    line = next((l for l in main_cpp.read_text(encoding="utf-8").splitlines() if "ultra->set_timing(" in l), "") if main_cpp else ""
    ns = int(line.rsplit(",", 1)[1].strip().rstrip(");").strip()) if line else 0   # (the lane's reset on the bus' grid: whole groups, rounded up)
    check(run.returncode == 0 and 500000 <= ns < 600000, f"a PARLIO lane: the reset on the bus grid is {ns} ns (500000 .. 600000) ({line.strip() or run.stdout[-300:] + run.stderr[-300:]})")


if "--no-codegen" not in sys.argv:
    codegen("alone", 1, 1, None, 500000)
    codegen("alone_long", 1, 1, 800, 800000)
    codegen("behind", 3, 3, None, 500000)
    codegen_parlio()
    codegen_descriptors()

print("chain reset floor: %s" % ("FAIL, %d" % failures if failures else "ok"))
sys.exit(1 if failures else 0)
