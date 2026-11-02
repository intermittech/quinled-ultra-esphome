"""The Diff-Power port's codegen helpers (components/diffpower/__init__.py) on hand-made configurations: the lights
that share pixels with a port's lights (partitions at any depth, the strips under them, a cycle), their turn-off
periods, and the lights diffpower.wake may name (design review, round 8); "During dark"'s source in each configuration
(Home Assistant's sun times, ESPHome's sun, both, none: the select's options) and what during_dark is refused without.
Needs ESPHome (the venv).

Run: python tests/diffpower_config_test.py"""

from pathlib import Path
import importlib.util
import sys

from esphome.config_validation import TimePeriod
from esphome.core import ID

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("diffpower_init", HERE.parent / "components" / "diffpower" / "__init__.py")
dp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dp)

failures = 0


def check(cond, what):
    global failures
    print(("ok   " if cond else "FAIL ") + what)
    failures += not cond


def strip(name):
    return {"platform": "esp32_rmt_led_strip", "id": ID(name)}


def partition(name, *sources):
    return {"platform": "partition", "id": ID(name), "segments": [{"id": ID(s), "from": 0, "to": 3} for s in sources]}


def entry(name, hold=None):
    e = {"id": ID(name)}
    if hold is not None:
        e["standby_after"] = TimePeriod(milliseconds=hold)
    return e


def full(*lights, ports=(), hubs=()):
    return {"light": list(lights), "diffpower": list(ports), "ultra_line": list(hubs)}


# strip <- part <- outer (a partition of a partition), and another strip with its own partition
cfg = full(strip("strip"), partition("part", "strip"), partition("outer", "part"), strip("other"),
           partition("other_part", "other"))
rel = dp._related_lights(cfg, [entry("strip")], 600000)
check(rel == {"part": 600000, "outer": 600000}, f"the strip listed: both partitions, nested too ({rel})")
rel = dp._related_lights(cfg, [entry("outer")], 600000)
check(rel == {"part": 600000, "strip": 600000}, f"the outer partition listed: down to the strip ({rel})")
rel = dp._related_lights(cfg, [entry("part", 3600000)], 600000)
check(rel == {"strip": 3600000, "outer": 3600000}, f"the middle one listed (1 h): both ways, its hold ({rel})")
rel = dp._related_lights(cfg, [entry("strip", 300000), entry("outer", 3600000)], 600000)
check(rel == {"part": 3600000}, f"two listed: the longest hold for what they share ({rel})")
check("other_part" not in dp._related_lights(cfg, [entry("strip")], 600000), "another strip's partition: not")

# a partition spanning two strips, one of them on the port: the partition is; the other strip isn't
cfg2 = full(strip("a"), strip("b"), partition("both", "a", "b"))
rel = dp._related_lights(cfg2, [entry("a")], 0)
check(rel == {"both": 0}, f"a partition over two strips: the port's, not the other strip ({rel})")

# a cycle (not a valid ESPHome configuration, but the search must stop)
cfg3 = full(strip("s"), partition("p1", "p2", "s"), partition("p2", "p1"))
rel = dp._related_lights(cfg3, [entry("s")], 0)
check(rel == {"p1": 0, "p2": 0}, f"a cycle: stops ({rel})")

# the wake targets: a port's lights, at any depth, and the lights of Ultra lines on a port
port = {"id": ID("p1"), "lights": [entry("strip")]}
hub_on = {"id": ID("h1"), "port": ID("p1")}
hub_off = {"id": ID("h2")}
cfg4 = full(strip("strip"), partition("part", "strip"), partition("outer", "part"),
            {"platform": "ur_analog_m", "id": ID("u1"), "ultra_line_id": ID("h1")},
            {"platform": "dig2analog_plus", "id": ID("u2"), "ultra_line_id": ID("h2")},
            ports=[port], hubs=[hub_on, hub_off])
on = dp._port_lights(cfg4)
check({"strip", "part", "outer", "u1"} <= on and "u2" not in on, f"wake targets: {sorted(on)}")

# "During dark"'s source in each configuration: Home Assistant's sun times (api: and a clock), ESPHome's sun (sun_id),
# both (the sun the backup), or none (the select without "During dark"); the clock: time_id, the sun's, the only time
ALL3, TWO = dp.MODE_OPTIONS, ["Standby", "Always on", "Off"]   # (TWO: without During dark)
SUN = {"id": ID("s"), "latitude": 52.1, "longitude": 5.1, "time_id": ID("t_sun")}


def dark(port_extra=None, api=False, times=(), sun=False):
    cfg = {"time": [{"id": ID(x)} for x in times]}
    if api:
        cfg["api"] = {}
    if sun:
        cfg["sun"] = SUN
    conf = {"power_mode": "standby", "sun_entity": "sun.sun", **(port_extra or {})}
    clock, ha, sun_conf = dp._dark_plan(conf, cfg)
    return (str(clock) if clock is not None else None), ha, sun_conf is not None, dp._mode_options(conf, cfg)


for what, args, want in [
    ("api, one time: Home Assistant's", dict(api=True, times=["ha_time"]), ("ha_time", True, False, ALL3)),
    ("api, no time: none", dict(api=True), (None, False, False, TWO)),
    ("api, two times: none (time_id needed)", dict(api=True, times=["a", "b"]), (None, False, False, TWO)),
    ("api, two times, time_id", dict(port_extra={"time_id": ID("b")}, api=True, times=["a", "b"]),
     ("b", True, False, ALL3)),
    ("api and a sun: Home Assistant's, the sun the backup (its clock)",
     dict(port_extra={"sun_id": ID("s")}, api=True, times=["t_sun", "other"], sun=True), ("t_sun", True, True, ALL3)),
    ("no api, a sun: the sun alone", dict(port_extra={"sun_id": ID("s")}, times=["t_sun"], sun=True),
     ("t_sun", False, True, ALL3)),
    ("no api, one time, no sun: none", dict(times=["t"]), ("t", False, False, TWO)),
    ("no api, no time: none", dict(), (None, False, False, TWO)),
]:
    got = dark(**args)
    check(got == want, f"{what}: {got}")

# the power mode Off (round 12): the 4th option, mode 3 (diffpower_core.h's PowerMode), "off" quoted or bare (YAML reads
# a bare off as false)
core = (HERE.parent / "components" / "diffpower" / "diffpower_core.h").read_text(encoding="utf-8")
check(dp.MODE_OPTIONS == ["Standby", "During dark", "Always on", "Off"] and
      list(dp.POWER_MODES.values()) == [0, 1, 2, 3] and
      "MODE_STANDBY = 0, MODE_DURING_DARK, MODE_ALWAYS_ON, MODE_OFF, MODE_COUNT" in core,
      "Off: option 4, mode 3, in the core's order")
check(dp._power_mode("off") == "off" and dp._power_mode(False) == "off" and dp._power_mode("Always_On") == "always_on",
      "power_mode: off, a bare off (false), any case")
try:
    dp._power_mode(True)
    check(False, "power_mode: a bare on: refused")
except Exception as e:  # cv.Invalid
    check(True, f"power_mode: a bare on: refused ('{e}')")

# power_mode: during_dark without a source: refused, with what is missing
for what, args, needle in [
    ("no api, no sun", dict(), "from Home Assistant (api:"),
    ("api, no time", dict(api=True), "needs the time now"),
    ("api, two times", dict(api=True, times=["a", "b"]), "say which one with time_id"),
]:
    cfg = {"time": [{"id": ID(x)} for x in args.get("times", [])], **({"api": {}} if args.get("api") else {})}
    try:
        dp._validate_dark({"power_mode": "during_dark"}, cfg)
        check(False, f"during_dark, {what}: refused")
    except Exception as e:  # cv.Invalid
        check(needle in str(e), f"during_dark, {what}: refused ('{e}')")
try:
    dp._validate_dark({"power_mode": "during_dark"}, {"api": {}, "time": [{"id": ID("t")}],
                                                       "diffpower": [{"sun_entity": "sun.sun"},
                                                                     {"sun_entity": "sun.other"}]})
    check(False, "two sun entities: refused")
except Exception as e:
    check("one for all ports" in str(e), f"two sun entities: refused ('{e}')")

print(f"{'ALL DIFFPOWER CONFIG TESTS PASSED' if not failures else 'FAILED'} ({failures} failures)")
sys.exit(1 if failures else 0)
