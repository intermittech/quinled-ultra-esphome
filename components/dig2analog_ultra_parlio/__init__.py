"""dig2analog_ultra_parlio: up to 16 dig2analog_ultra data lines from the ESP32-P4's PARLIO TX unit (one DMA stream).

Each `dig2analog_ultra:` hub with `parlio_id` and `lane` sends on the bus' pin of that lane instead of an RMT channel.
The hubs keep everything else of their own (scheduler, TIME, group start, preamble, refresh, telemetry); every bit
of the bus takes one period on its sample grid (dig2analog_ultra/parlio_timing.py, rendered by ultra_parlio.h).
"""

from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32
from esphome.components.esp32 import add_idf_sdkconfig_option, include_builtin_idf_component
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import CONF_ID, CONF_INVERTED, CONF_NUMBER, CONF_PINS
from esphome.core import CORE

from esphome.components.dig2analog_ultra.parlio_timing import PARLIO_CLOCK_HZ, layout, rate_ok

DEPENDENCIES = ["esp32", "dig2analog_ultra"]
MULTI_CONF = True

CONF_SAMPLE_RATE = "sample_rate"
CONF_BUFFER_TIME = "buffer_time"
CONF_PSRAM = "psram"
CONF_PARLIO_ID = "parlio_id"
CONF_LANE = "lane"

dig2analog_ultra_ns = cg.esphome_ns.namespace("dig2analog_ultra")
UltraParlioBus = dig2analog_ultra_ns.class_("UltraParlioBus", cg.Component)

# The chips with a PARLIO TX unit of 16 lines that ESP-IDF 5.5 supports; the ESP32-S31 (16-bit PARLIO in its
# preliminary datasheet) once ESP-IDF supports it.
PARLIO_VARIANTS = [esp32.VARIANT_ESP32P4]
MAX_LANES = 16
DEFAULT_TIMING = (250, 1050, 700, 650, 300000)   # the hub's defaults (ns)


def _validate_rate(value):
    hz = int(cv.frequency(value))
    if not rate_ok(hz):
        raise cv.Invalid(
            f"sample_rate must divide the PARLIO clock ({PARLIO_CLOCK_HZ // 1_000_000} MHz) with a whole number of ns "
            "per sample: 8, 10, 20 (default) or 40 MHz"
        )
    return hz


def _validate_pins(value):
    value = cv.ensure_list(pins.internal_gpio_output_pin_schema)(value)
    if not 1 <= len(value) <= MAX_LANES:
        raise cv.Invalid(f"pins: 1-{MAX_LANES} data pins (lane 0 first)")
    return value


CONFIG_SCHEMA = cv.All(
    esp32.only_on_variant(supported=PARLIO_VARIANTS, msg_prefix="dig2analog_ultra_parlio (PARLIO TX, 16 lines)"),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraParlioBus),
            # the data pins: lane 0, 1, ... (a hub's `lane:` is its index here)
            cv.Required(CONF_PINS): _validate_pins,
            # the sample clock: 20 MHz (50 ns) keeps the default bit timing exact; 10 MHz rounds '0' to 300 ns
            cv.Optional(CONF_SAMPLE_RATE, default="20MHz"): _validate_rate,
            # one DMA buffer's time (two take turns): frames start up to ~3 of them ahead of now; a longer one rides
            # out a busier CPU, a shorter one saves RAM. Default: 1 ms (longer where the DMA's look-ahead needs it)
            cv.Optional(CONF_BUFFER_TIME): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(min=cv.TimePeriod(microseconds=250), max=cv.TimePeriod(milliseconds=10)),
            ),
            # the two buffers in PSRAM (internal RAM by default; the other one when it has no room)
            cv.Optional(CONF_PSRAM, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


def lanes_of(full, bus_id):
    """The hubs on this bus: {lane: hub config}."""
    hubs = full.get("dig2analog_ultra", []) or []
    return {
        h[CONF_LANE]: h
        for h in (hubs if isinstance(hubs, list) else [hubs])
        if str(h.get(CONF_PARLIO_ID, "")) == str(bus_id)
    }


def hub_timing(hub):
    """(bit0_high, bit0_low, bit1_high, bit1_low, reset) of a hub, ns."""
    return tuple(int(hub[k].total_nanoseconds) for k in ("bit0_high", "bit0_low", "bit1_high", "bit1_low", "reset_time"))


def buffer_ns(bus):
    return int(bus[CONF_BUFFER_TIME].total_microseconds) * 1000 if CONF_BUFFER_TIME in bus else None


def bus_layout(bus, full):
    lanes = lanes_of(full, bus[CONF_ID])
    return layout(bus[CONF_SAMPLE_RATE], len(bus[CONF_PINS]), {lane: hub_timing(h) for lane, h in lanes.items()},
                  buffer_ns(bus))


def find_bus(full, bus_id):
    for bus in full.get("dig2analog_ultra_parlio", []) or []:
        if str(bus[CONF_ID]) == str(bus_id):
            return bus
    return None


def _final_validate(configs):
    full = fv.full_config.get()
    every = full.get("dig2analog_ultra_parlio", []) or []   # (ESPHome calls this per bus)
    if len(every) > 1:
        raise cv.Invalid(
            f"the ESP32-P4 has one PARLIO TX unit: one dig2analog_ultra_parlio (up to {MAX_LANES} pins), not "
            f"{len(every)}"
        )
    for bus in configs if isinstance(configs, list) else [configs]:
        lay = bus_layout(bus, full)
        if lay["errors"]:
            raise cv.Invalid(f"dig2analog_ultra_parlio '{bus[CONF_ID]}': " + "; ".join(lay["errors"]))
    return configs


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    include_builtin_idf_component("esp_driver_gptimer")
    # the timer's interrupt (the direction pins' releases, the render deadline) also runs while flash writes have the
    # cache off, and so do the GPTimer functions it calls
    add_idf_sdkconfig_option("CONFIG_GPTIMER_ISR_CACHE_SAFE", True)
    add_idf_sdkconfig_option("CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM", True)
    cg.add_define("USE_DIG2ANALOG_ULTRA_PARLIO")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    lay = bus_layout(config, CORE.config)
    if not lay["highs"]:   # no hub on it: the stream idles on the default timing's grid
        lay = layout(config[CONF_SAMPLE_RATE], len(config[CONF_PINS]), {0: DEFAULT_TIMING}, buffer_ns(config))
        lay["highs"] = {}
    for lane, pin in enumerate(config[CONF_PINS]):
        cg.add(var.set_pin(lane, pin[CONF_NUMBER], pin[CONF_INVERTED]))
    for lane, (h0, h1) in lay["highs"].items():
        cg.add(var.set_lane_timing(lane, h0, h1))
    cg.add(var.set_layout(config[CONF_SAMPLE_RATE], lay["width"], lay["period"], lay["chunk_groups"],
                          lay["margin_ns"], config[CONF_PSRAM]))
    # only the render path this layout takes is built (ultra_parlio.h)
    cg.add_define("PARLIO_ONLY_WIDTH", lay["width"])
    cg.add_define("PARLIO_ONLY_GRID", lay["grid"] if lay["highs"] else 0)   # (no lane: nothing renders)
