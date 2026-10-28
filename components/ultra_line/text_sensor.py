"""The line speed of one line (ultra_line) as a Home Assistant diagnostic: optional, none unless configured.

Its profile, a refused faster profile and why (for example a Dig2Analog-Plus-5ch pin whose boards' firmware lacks the
speed its board count needs), and what the line is doing about it. A Diff-Power port's receivers show the same text;
a pin without a return path has only this one.
"""

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_ULTRA_LINE_ID, UltraHub

DEPENDENCIES = ["ultra_line"]

CONF_LINE_SPEED = "line_speed"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ULTRA_LINE_ID): cv.use_id(UltraHub),
        cv.Optional(CONF_LINE_SPEED): text_sensor.text_sensor_schema(
            icon="mdi:speedometer", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ULTRA_LINE_ID])
    if CONF_LINE_SPEED in config:
        cg.add(hub.set_line_speed_sensor(await text_sensor.new_text_sensor(config[CONF_LINE_SPEED])))
