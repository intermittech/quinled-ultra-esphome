"""The reset floor of a normal Ultra line whose first board is an ultra_digital receiver (components/dig2analog_ultra/chain_reset.py): the hub's
reset_time is raised to 500 us in that case and left alone in every other (a receiver behind another board, a digital port, a PARLIO lane,
no receiver, a larger reset). Plain Python: the module has no ESPHome imports.

then the codegen of three small configurations (esphome compile --only-generate: the hub's set_timing in main.cpp). Needs ESPHome for
the second part (the venv); --no-codegen leaves it out.

Run: python tests/run_ultra_chain_reset_test.py [--no-codegen]"""

from pathlib import Path
import importlib.util
import subprocess
import sys

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("chain_reset", HERE.parent / "components" / "dig2analog_ultra" / "chain_reset.py")
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


def rx(hub_name, board, platform="ultra_digital"):
    return {"platform": platform, "dig2analog_ultra_id": Id(hub_name), "board": board}


check(cr.CHAIN_FIRST_RESET_NS == 500000, "the floor is 500 us")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 0)]) == 500000, "a receiver alone on a normal line: the default 300 us becomes 500 us")
check(cr.reset_with_floor_ns(280001, hub(), [rx("u", 0)]) == 500000, "... and the shortest allowed reset too")
check(cr.reset_with_floor_ns(800000, hub(), [rx("u", 0)]) == 800000, "a longer reset stays")
check(cr.reset_with_floor_ns(500000, hub(), [rx("u", 0)]) == 500000, "exactly 500 us stays")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 2)]) == 300000, "a receiver behind other boards (board 2): nothing changes (320 us a board more gap)")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 1)]) == 300000, "... board 1 too")
check(cr.reset_with_floor_ns(300000, hub(), [rx("other", 0)]) == 300000, "a receiver on another hub: nothing changes")
check(cr.reset_with_floor_ns(300000, hub(), [rx("u", 0, "dig2analog_ultra")]) == 300000, "a dig2analog_ultra light on board 0: nothing changes")
check(cr.reset_with_floor_ns(300000, hub(), []) == 300000, "no receiver: nothing changes")
check(cr.reset_with_floor_ns(300000, hub(), None) == 300000, "no lights at all: nothing changes")
check(cr.reset_with_floor_ns(300000, hub(digital={"reset_time": 300000}), [rx("u", 0)]) == 300000, "a digital port (the receiver is in a strip's data path): nothing changes")
check(cr.reset_with_floor_ns(300000, hub(parlio_id=Id("bus")), [rx("u", 0)]) == 300000, "a PARLIO lane (its timing is the bus' grid): nothing changes")
check(cr.chain_first_receiver(hub(), [rx("u", 1), rx("u", 0)]), "any of the hub's receivers on board 0 counts")

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
    components: [dig2analog_ultra, ultra_digital]
logger:
dig2analog_ultra:
  - id: ultra
    pin: GPIO16
    num_boards: {boards}
{extra}light:
  - platform: ultra_digital
    id: bar
    name: Bar
    dig2analog_ultra_id: ultra
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


if "--no-codegen" not in sys.argv:
    codegen("alone", 1, 0, None, 500000)
    codegen("alone_long", 1, 0, 800, 800000)
    codegen("behind", 3, 2, None, 300000)

print("chain reset floor: %s" % ("FAIL, %d" % failures if failures else "ok"))
sys.exit(1 if failures else 0)
