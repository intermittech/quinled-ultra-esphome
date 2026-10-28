"""Diagnostics of one line (ultra_line) (ultra_diagnostics.h): optional sensors, none unless configured.

The ESP32's own heap low-water mark is ESPHome's `debug:` sensor `min_free`; these are the line's share.
"""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_BYTES,
)

from . import CONF_ULTRA_LINE_ID, UltraHub, ultra_line_ns

DEPENDENCIES = ["ultra_line"]

CONF_QUEUE_PEAK = "queue_peak"
CONF_MEMORY_PEAK = "memory_peak"
CONF_FRAME_BUILD_TIME = "frame_build_time"
CONF_RMT_UNDERRUNS = "rmt_underruns"
CONF_FRAME_RATE = "frame_rate"

UltraDiagnostics = ultra_line_ns.class_("UltraDiagnostics", cg.PollingComponent)


def _diagnostic(unit, icon, decimals=0, state_class=STATE_CLASS_MEASUREMENT):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        icon=icon,
        accuracy_decimals=decimals,
        state_class=state_class,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraDiagnostics),
        cv.GenerateID(CONF_ULTRA_LINE_ID): cv.use_id(UltraHub),
        # the most commands waiting for one board since the last update (a full line: they keep growing)
        cv.Optional(CONF_QUEUE_PEAK): _diagnostic("commands", "mdi:tray-full"),
        # the line's heap at its highest since the last update: boards, lights, effect steps and the queues
        cv.Optional(CONF_MEMORY_PEAK): _diagnostic(UNIT_BYTES, "mdi:memory"),
        # the longest frame build since the last update
        cv.Optional(CONF_FRAME_BUILD_TIME): _diagnostic("µs", "mdi:timer-outline"),
        # frames the RMT ran out of symbols in (they were broken) since boot; counting costs ~1 us per RMT refill
        cv.Optional(CONF_RMT_UNDERRUNS): _diagnostic(
            cv.UNDEFINED, "mdi:alert-circle-outline", 0, STATE_CLASS_TOTAL_INCREASING
        ),
        # frames sent per second (a 50-board line is full at ~58)
        cv.Optional(CONF_FRAME_RATE): _diagnostic("frames/s", "mdi:speedometer", 1),
    }
).extend(cv.polling_component_schema("60s"))

SENSORS = {
    CONF_QUEUE_PEAK: "set_queue_peak",
    CONF_MEMORY_PEAK: "set_memory_peak",
    CONF_FRAME_BUILD_TIME: "set_frame_build_time",
    CONF_RMT_UNDERRUNS: "set_rmt_underruns",
    CONF_FRAME_RATE: "set_frame_rate",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_hub(await cg.get_variable(config[CONF_ULTRA_LINE_ID])))
    for key, setter in SENSORS.items():
        if key in config:
            cg.add(getattr(var, setter)(await sensor.new_sensor(config[key])))
