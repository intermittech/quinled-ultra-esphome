"""ultra_digital_stream: an ESPHome addressable light on the strip behind a QuinLED Diff-Power Ultra-Digital receiver.

The light's pixel frames go through the digital port's hub (dig2analog_ultra with digital:): it sends them between its
marker frames (commands, polls) and stops them while the receiver shows a local effect (ultra_digital). The strip's chip
gives the pixel frames their timing; the receiver passes them on unchanged, delayed by its delay line.
"""

from dataclasses import dataclass

import esphome.codegen as cg
from esphome.components import light
from esphome.components.const import CONF_CHANNEL_COLORS
from esphome.components.dig2analog_ultra import CONF_DIG2ANALOG_ULTRA_ID, UltraHub
import esphome.config_validation as cv
from esphome.const import CONF_CHIPSET, CONF_ID, CONF_NUM_LEDS, CONF_OUTPUT_ID
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@intermittech"]
DEPENDENCIES = ["dig2analog_ultra"]

ultra_digital_stream_ns = cg.esphome_ns.namespace("ultra_digital_stream")
UltraDigitalStreamOutput = ultra_digital_stream_ns.class_("UltraDigitalStreamOutput", light.AddressableLight)

CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_RESET_TIME = "reset_time"


@dataclass
class ChipTiming:
    """ns; the reset in us. The centres of the receiver's chip table (a pixel is a '0' below the mean of the two highs)."""

    bit0_high: int
    bit0_low: int
    bit1_high: int
    bit1_low: int
    reset_us: int


CHIPSETS = {
    "WS2811": ChipTiming(300, 950, 900, 350, 300),          # 800 kHz
    "WS2811_400K": ChipTiming(500, 2000, 1200, 1300, 80),   # 400 kHz
    "WS2812": ChipTiming(400, 850, 800, 450, 300),
    "WS2815": ChipTiming(400, 850, 800, 450, 300),
    "SK6812": ChipTiming(300, 950, 600, 650, 80),
    "WS2805": ChipTiming(300, 790, 790, 300, 300),
}


def _reset_us(value):
    value = cv.positive_time_period_nanoseconds(value)
    us = value.total_nanoseconds / 1000
    if us < 50 or us > 800:
        raise cv.Invalid("reset_time: 50us to 800us")
    return value


def _final_validate(config: ConfigType) -> ConfigType:
    full = fv.full_config.get()
    hub = next((h for h in full.get("dig2analog_ultra", []) or [] if h[CONF_ID] == config[CONF_DIG2ANALOG_ULTRA_ID]), None)
    if hub is None or "digital" not in hub:
        raise cv.Invalid(
            "the data line is not a digital port: give the dig2analog_ultra line digital: (and num_boards: 1)",
            [CONF_DIG2ANALOG_ULTRA_ID],
        )
    same = [
        c
        for c in full.get("light", []) or []
        if c.get("platform") == "ultra_digital_stream" and c.get(CONF_DIG2ANALOG_ULTRA_ID) == config[CONF_DIG2ANALOG_ULTRA_ID]
    ]
    if len(same) > 1:
        raise cv.Invalid(f"the digital port '{hub[CONF_ID]}' has {len(same)} stream lights; one strip, one stream light")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

CONFIG_SCHEMA = cv.All(
    light.ADDRESSABLE_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(UltraDigitalStreamOutput),
            cv.GenerateID(CONF_DIG2ANALOG_ULTRA_ID): cv.use_id(UltraHub),
            cv.Required(CONF_NUM_LEDS): cv.int_range(min=1, max=2500),
            cv.Optional(CONF_CHANNEL_COLORS, default="GRB"): light.validate_channel_colors,
            cv.Optional(CONF_CHIPSET): cv.one_of(*CHIPSETS, upper=True),
            cv.Inclusive(CONF_BIT0_HIGH, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT0_LOW, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT1_HIGH, "custom"): cv.positive_time_period_nanoseconds,
            cv.Inclusive(CONF_BIT1_LOW, "custom"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_RESET_TIME): _reset_us,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_exactly_one_key(CONF_CHIPSET, CONF_BIT0_HIGH),
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)
    await cg.register_component(var, config)
    hub = await cg.get_variable(config[CONF_DIG2ANALOG_ULTRA_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_num_leds(config[CONF_NUM_LEDS]))
    cg.add(var.set_channel_colors(light.channel_colors_struct(config[CONF_CHANNEL_COLORS])))
    if CONF_CHIPSET in config:
        t = CHIPSETS[config[CONF_CHIPSET]]
        timing = (t.bit0_high, t.bit0_low, t.bit1_high, t.bit1_low, t.reset_us)
    else:
        timing = (
            int(config[CONF_BIT0_HIGH].total_nanoseconds),
            int(config[CONF_BIT0_LOW].total_nanoseconds),
            int(config[CONF_BIT1_HIGH].total_nanoseconds),
            int(config[CONF_BIT1_LOW].total_nanoseconds),
            300,
        )
    if CONF_RESET_TIME in config:
        timing = timing[:4] + (int(config[CONF_RESET_TIME].total_nanoseconds // 1000),)
    bytes_per_led = 4 if "W" in config[CONF_CHANNEL_COLORS] else 3
    cg.add(hub.set_digital_stream(config[CONF_NUM_LEDS] * bytes_per_led, *timing))
