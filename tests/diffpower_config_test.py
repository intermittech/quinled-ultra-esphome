"""The Diff-Power port's codegen helpers (components/diffpower/__init__.py) on hand-made configurations: the lights
that share pixels with a port's lights (partitions at any depth, the strips under them, a cycle), their turn-off
periods, and the lights diffpower.wake may name (review-r8/portmgr-recheck-gpt N5). Needs ESPHome (the venv).

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
    return {"light": list(lights), "diffpower": list(ports), "dig2analog_ultra": list(hubs)}


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
            {"platform": "dig2analog_ultra", "id": ID("u1"), "dig2analog_ultra_id": ID("h1")},
            {"platform": "dig2analog_ultra", "id": ID("u2"), "dig2analog_ultra_id": ID("h2")},
            ports=[port], hubs=[hub_on, hub_off])
on = dp._port_lights(cfg4)
check({"strip", "part", "outer", "u1"} <= on and "u2" not in on, f"wake targets: {sorted(on)}")

print(f"{'ALL DIFFPOWER CONFIG TESTS PASSED' if not failures else 'FAILED'} ({failures} failures)")
sys.exit(1 if failures else 0)
