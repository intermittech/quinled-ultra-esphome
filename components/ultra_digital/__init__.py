"""ultra_digital: the QuinLED Diff-Power Ultra-Digital receiver's local pixel effects (light platform: light.py).

The receiver makes the strip's pixels itself (the PIXEL_FX command of the QuinLED Ultra protocol 0.2.8): Home Assistant
users get WLED-like effects, speed, intensity and palettes without a pixel stream. The effect list is the protocol's
catalogue (pixel_fx_catalogue.json). This file: the actions and what the platform shares (chips, the catalogue).
"""

from esphome import automation
import esphome.codegen as cg
from esphome.components import light
import esphome.config_validation as cv
from esphome.const import CONF_ID

from . import catalogue

CODEOWNERS = ["@intermittech"]
DEPENDENCIES = ["dig2analog_ultra"]
# the Speed / Intensity / Custom numbers, the Palette select and the status text
AUTO_LOAD = ["number", "select", "text_sensor"]

ultra_digital_ns = cg.esphome_ns.namespace("ultra_digital")
EffectParamsAction = ultra_digital_ns.class_("EffectParamsAction", automation.Action)

CONF_COLOR_2 = "color_2"
CONF_COLOR_3 = "color_3"
CONF_SPEED = "speed"
CONF_INTENSITY = "intensity"
CONF_CUSTOM1 = "custom1"
CONF_CUSTOM2 = "custom2"
CONF_PALETTE = "palette"

# The chips of `chip:`: name -> (index into ultra_pixel_fx.h PIXEL_CHIPS, bytes per pixel, the chip's own wire order).
# (tests/run_ultra_digital_test.py checks the three against the header's table)
CHIPS = {
    "ws2812b": (0, 3, "GRB", "ULTRA_PXC_WS2812B_V5"),             # V5 / 2020 and later, WS2811 V2.2 and later (800 kHz)
    "ws2812b_classic": (1, 3, "GRB", "ULTRA_PXC_WS2812B_CLASSIC"),   # before V5
    "sk6812": (2, 3, "GRB", "ULTRA_PXC_SK6812_RGB"),              # SK6812 RGB
    "sk6812_rgbw": (3, 4, "GRBW", "ULTRA_PXC_SK6812_RGBW"),
    "ws2814": (4, 4, "RGBW", "ULTRA_PXC_WS2814_RGBW"),            # 24 V; WRGB and others through color_order
    "ws2805": (5, 5, "RGBWC", "ULTRA_PXC_WS2805_RGBCCT"),         # RGB + warm white + cold white
    "ws2815": (6, 3, "GRB", "ULTRA_PXC_WS2815"),                  # 12 V with a backup data line
    "ws2811_400k": (7, 3, "RGB", "ULTRA_PXC_WS2811_400K"),        # WS2811 at 400 kHz
}

PIXEL_FX_SEGMENTS = 4   # ULTRA_PX_SEGMENTS


def rgb_value(value):
    """A colour: 0xRRGGBB, '#RRGGBB' or [r, g, b]"""
    if isinstance(value, str):
        text = value.strip().lstrip("#")
        try:
            value = int(text, 16)
        except ValueError as err:
            raise cv.Invalid(f"'{value}' is not a colour (0xRRGGBB, '#RRGGBB' or [r, g, b])") from err
        if len(text) != 6:
            raise cv.Invalid(f"'{value}': six hex digits RRGGBB")
    if isinstance(value, (list, tuple)):
        if len(value) != 3:
            raise cv.Invalid("a colour list is [red, green, blue]")
        r, g, b = (cv.int_range(min=0, max=255)(v) for v in value)
        return (r << 16) | (g << 8) | b
    return cv.int_range(min=0, max=0xFFFFFF)(value)


def palette_index(value):
    """A palette option of the catalogue by name (case-insensitive) or index (0 = the effect's own, 1 = c1 -> c2 -> c3)"""
    options = catalogue.load().palette_options()
    if isinstance(value, int):
        return cv.int_range(min=0, max=len(options) - 1)(value)
    text = cv.string_strict(value).lower()
    for i, name in enumerate(options):
        if name.lower() == text:
            return i
    raise cv.Invalid(f"unknown palette '{value}': {', '.join(options)}")


EFFECT_PARAMS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(light.LightState),
        cv.Optional(CONF_COLOR_2): cv.templatable(rgb_value),
        cv.Optional(CONF_COLOR_3): cv.templatable(rgb_value),
        cv.Optional(CONF_SPEED): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_INTENSITY): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_CUSTOM1): cv.templatable(cv.int_range(min=0, max=255)),
        cv.Optional(CONF_CUSTOM2): cv.templatable(cv.int_range(min=0, max=255)),
        # the Palette select's option: a number, or a name (not templatable)
        cv.Optional(CONF_PALETTE): cv.templatable(palette_index),
    }
)


@automation.register_action(
    "ultra_digital.effect_params", EffectParamsAction, EFFECT_PARAMS_SCHEMA, synchronous=True
)
async def effect_params_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    cg.add(var.set_light(await cg.get_variable(config[CONF_ID])))
    for key, setter, ctype in (
        (CONF_COLOR_2, "set_color_2", cg.uint32),
        (CONF_COLOR_3, "set_color_3", cg.uint32),
        (CONF_SPEED, "set_speed", cg.uint8),
        (CONF_INTENSITY, "set_intensity", cg.uint8),
        (CONF_CUSTOM1, "set_custom1", cg.uint8),
        (CONF_CUSTOM2, "set_custom2", cg.uint8),
        (CONF_PALETTE, "set_palette", cg.uint8),
    ):
        if key in config:
            cg.add(getattr(var, setter)(await cg.templatable(config[key], args, ctype)))
    return var
