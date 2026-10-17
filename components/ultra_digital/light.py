"""ultra_digital light: one segment of a QuinLED Diff-Power Ultra-Digital receiver's strip, effects run by the receiver.

The effect list is the protocol's catalogue (generated at compile time from pixel_fx_catalogue.json); Speed,
Intensity and Palette are entities of the light; c2, c3 and the extra values go through the ultra_digital.effect_params
action. ESPHome's transition length goes to the receiver, which ramps brightness and colours itself.

The receiver is on one of two kinds of line: a digital port (digital: on the data line; the receiver sits in the strip's
data path, mode stream) or a normal Ultra line as its last board (chain mode, protocol 0.2.9). `mode:` says which
(CONFIG 140); auto lets the line decide.
"""

import logging

import esphome.codegen as cg
from esphome.components import light, number, select, text_sensor
from esphome.components.dig2analog_ultra import CONF_DIG2ANALOG_ULTRA_ID, CONF_NUM_BOARDS, UltraHub
from esphome.components.dig2analog_ultra.receiver_schema import CONF_TELEMETRY
from esphome.components.light.effects import register_rgb_effect
from esphome.components.light.types import LightEffect
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEVICE_ID,
    CONF_EFFECTS,
    CONF_ID,
    CONF_NAME,
    CONF_OUTPUT_ID,
    CONF_TYPE_ID,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import (
    CHIPS,
    CONF_COLOR_2,
    CONF_COLOR_3,
    CONF_CUSTOM1,
    CONF_CUSTOM2,
    CONF_INTENSITY,
    CONF_PALETTE,
    CONF_SPEED,
    PIXEL_FX_SEGMENTS,
    catalogue,
    palette_index,
    rgb_value,
    ultra_digital_ns,
)

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["dig2analog_ultra"]
AUTO_LOAD = ["number", "select", "text_sensor"]

PixelLight = ultra_digital_ns.class_("PixelLight", light.LightOutput, cg.Component)
PixelFxEffect = ultra_digital_ns.class_("PixelFxEffect", LightEffect)
PixelNumber = ultra_digital_ns.class_("PixelNumber", number.Number, cg.Component)
PixelPalette = ultra_digital_ns.class_("PixelPalette", select.Select, cg.Component)

CONF_BOARD = "board"
CONF_SEGMENT = "segment"
CONF_CHIP = "chip"
CONF_NUM_LEDS = "num_leds"
CONF_COLOR_ORDER = "color_order"
CONF_SEGMENT_START = "segment_start"
CONF_SEGMENT_LENGTH = "segment_length"
CONF_REVERSE = "reverse"
CONF_MIRROR = "mirror"
CONF_LOCK = "lock"
CONF_AUTO_WHITE = "auto_white"
CONF_PIXEL_SMOOTHING = "pixel_smoothing"
CONF_COLD_WHITE_SHARE = "cold_white_share"
CONF_MAX_FPS = "max_fps"
CONF_LED_CURRENT = "led_current"
CONF_CURRENT_LIMIT = "current_limit"
CONF_VIRTUAL_OFFSET = "virtual_offset"
CONF_VIRTUAL_LENGTH = "virtual_length"
CONF_STANDALONE = "standalone"
CONF_POWER_ON = "power_on"
CONF_ON_SIGNAL_LOSS = "on_signal_loss"
CONF_STREAM_WINS = "stream_wins"
CONF_SIGNAL_LOSS_TIMEOUT = "signal_loss_timeout"
CONF_MODE = "mode"
CONF_STORE_AFTER = "store_after"
CONF_STANDBY_AFTER = "standby_after"
CONF_EXCLUSIVE_WITH = "exclusive_with"
CONF_CONTROLS = "controls"
CONF_CUSTOM_CONTROLS = "custom_controls"
CONF_STATUS = "status"
CONF_FX = "fx"
CONF_CATALOGUE_EFFECTS = "catalogue_effects"

# CONFIG ids of the receiver's settings (ultra_protocol.h ULTRA_CFG_PX_*)
CFG_CHIP, CFG_ORDER, CFG_PIXELS, CFG_FPS, CFG_MA_CH, CFG_LIMIT_MA, CFG_WHITE = 129, 130, 131, 132, 133, 134, 135
CFG_VOFFSET, CFG_VLENGTH, CFG_POLICY, CFG_LOSS_TIMEOUT = 136, 137, 138, 139
# CONFIG 140, the line mode (ULTRA_PXLM_*): auto = the receiver decides once per power-up, stream = a digital port's receiver
# (in a strip's data path, the hub's pixels pass), chain = the last board of a normal Ultra line
LINE_MODES = {"auto": 0, "stream": 1, "chain": 2}
# CONFIG 141, the pixel smoothing (ULTRA_PXS_*): how the receiver takes its colour to the strip's 8 bits. auto (the default) = 16 bit gamma /
# brightness / auto-white, dithered at 50 frames a second or more and rounded below; round; dither; off = the 8-bit path of protocol 0.2.8
CFG_SMOOTH = 141
PIXEL_SMOOTHING = {"auto": 0, "round": 1, "dither": 2, "off": 3}


def _smoothing(value):
    """auto | round | dither | off (YAML reads an unquoted `off` as false)"""
    return "off" if value is False else cv.one_of(*PIXEL_SMOOTHING, lower=True)(value)

# the auto-white modes (ULTRA_PXW_*)
AUTO_WHITE = {"default": 0, "off": 1, "accurate": 2, "bright": 3}
# ULTRA_PXF_* (PIXEL_FX FLAGS)
PXF_REVERSE, PXF_MIRROR, PXF_LOCK = 0x01, 0x02, 0x20
# ULTRA_PXP_* (CONFIG 138)
PXP_POWER_ON, PXP_ON_LOSS, PXP_STREAM_WINS = 0x01, 0x02, 0x04
# the channels of a colour order (ULTRA_PXCH_*)
CHANNEL = {"R": 0, "G": 1, "B": 2, "W": 3, "C": 4}

PIXEL_FX_EFFECT = "ultra_digital_fx"


# ------------------------------------------------------------------------------------------------------ the effects
@register_rgb_effect(PIXEL_FX_EFFECT, PixelFxEffect, "Effect", {cv.Required(CONF_FX): cv.int_range(min=1, max=255)})
async def pixel_fx_effect_to_code(config, effect_id):
    return cg.new_Pvariable(effect_id, config[CONF_NAME], config[CONF_FX])


def _color_order(chip: str, value) -> tuple:
    """(the text, CONFIG 130's value): 0 = the chip's own order"""
    _, nbytes, own, _ = CHIPS[chip]
    text = cv.string_strict(value).upper()
    if len(text) != nbytes or any(c not in "RGBWC"[: 3 + (nbytes > 3) + (nbytes > 4)] for c in text) or len(set(text)) != nbytes:
        letters = "RGB" + ("W" if nbytes > 3 else "") + ("C" if nbytes > 4 else "")
        raise cv.Invalid(
            f"color_order '{value}' for {chip}: {nbytes} letters, each of {letters} once (the chip's own: {own})"
        )
    if text == own:
        return text, 0
    packed = 0
    for k, c in enumerate(text):
        packed |= CHANNEL[c] << (3 * k)
    return text, packed


def _catalogue_effects():
    cat = catalogue.load()
    return [{PIXEL_FX_EFFECT: {CONF_NAME: e["name"], CONF_FX: e["id"]}} for e in sorted(cat.effects, key=lambda e: e["id"])]


def _with_catalogue_effects(value):
    """The light's effect list is the catalogue's (catalogue_effects: false: only the ones listed); the ones listed
    come after (custom ids, 128-255, or a catalogue effect under another name)"""
    if not isinstance(value, dict):
        return value
    value = dict(value)
    if value.get(CONF_CATALOGUE_EFFECTS, True):
        listed = value.get(CONF_EFFECTS)
        value[CONF_EFFECTS] = _catalogue_effects() + (list(listed) if isinstance(listed, list) else [])
    return value


def _validate_range(config: ConfigType) -> ConfigType:
    start, length = config[CONF_SEGMENT_START], config[CONF_SEGMENT_LENGTH]
    if length and start + length > config[CONF_NUM_LEDS]:
        raise cv.Invalid(
            f"segment_start {start} + segment_length {length} is beyond num_leds ({config[CONF_NUM_LEDS]})",
            [CONF_SEGMENT_LENGTH],
        )
    if start >= config[CONF_NUM_LEDS]:
        raise cv.Invalid(f"segment_start {start} is beyond num_leds ({config[CONF_NUM_LEDS]})", [CONF_SEGMENT_START])
    chip = config[CONF_CHIP]
    config[CONF_COLOR_ORDER] = _color_order(chip, config.get(CONF_COLOR_ORDER, CHIPS[chip][2]))
    if config.get(CONF_COLD_WHITE_SHARE) is not None and chip != "ws2805":
        raise cv.Invalid("cold_white_share is for the WS2805 (warm and cold white)", [CONF_COLD_WHITE_SHARE])
    return config


CONFIG_SCHEMA = cv.All(
    _with_catalogue_effects,
    light.RGB_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(PixelLight),
            cv.GenerateID(CONF_DIG2ANALOG_ULTRA_ID): cv.use_id(UltraHub),
            # the receiver's position on the line (0 = the first after the controller; one receiver per digital port)
            cv.Optional(CONF_BOARD, default=0): cv.int_range(min=0, max=63),
            # up to four lights share a receiver's strip, each its own segment (pixel range); a higher segment is drawn
            # over a lower one
            cv.Optional(CONF_SEGMENT, default=0): cv.int_range(min=0, max=PIXEL_FX_SEGMENTS - 1),
            cv.Required(CONF_CHIP): cv.one_of(*CHIPS, lower=True),
            # the strip's pixels (every light of the receiver says the same); the receiver's limit follows from the
            # chip's bytes and the line's rate (page 0x81 reports it)
            cv.Required(CONF_NUM_LEDS): cv.int_range(min=1, max=2500),
            # the wire order of the chip's channels, e.g. GRB, RGBW, WRGB; WS2805: RGBWC (W warm, C cold)
            cv.Optional(CONF_COLOR_ORDER): cv.string_strict,
            cv.Optional(CONF_SEGMENT_START, default=0): cv.int_range(min=0, max=2499),
            cv.Optional(CONF_SEGMENT_LENGTH, default=0): cv.int_range(min=0, max=2500),   # 0 = to the end
            cv.Optional(CONF_REVERSE, default=False): cv.boolean,
            cv.Optional(CONF_MIRROR, default=False): cv.boolean,
            # a plain pixel stream from the controller doesn't take the strip back from this light's effect
            cv.Optional(CONF_LOCK, default=False): cv.boolean,
            # the white of RGBW and RGBCCT strips from the colour: off, accurate (taken out of R G B), bright
            cv.Optional(CONF_AUTO_WHITE): cv.one_of(*[k for k in AUTO_WHITE if k != "default"], lower=True),
            cv.Optional(CONF_COLD_WHITE_SHARE): cv.percentage,   # WS2805: the share of the cold white (default even)
            # smooth dimming (CONFIG 141): auto (default), round, dither or off (the 8-bit path); a receiver of an older firmware ignores it
            cv.Optional(CONF_PIXEL_SMOOTHING): _smoothing,
            cv.Optional(CONF_MAX_FPS): cv.int_range(min=0, max=1000),   # 0 = as many as the line allows
            # the strip's LED current per channel at full (for the current limit), and the limit: 0 = the link's
            cv.Optional(CONF_LED_CURRENT): cv.All(cv.current, cv.Range(min=0.001, max=0.1)),
            cv.Optional(CONF_CURRENT_LIMIT): cv.Any(
                cv.one_of(0), cv.All(cv.current, cv.Range(min=0.1, max=65.0))
            ),
            # an effect continuing over several strips: this strip starts at virtual_offset of a virtual strip
            cv.Optional(CONF_VIRTUAL_OFFSET): cv.int_range(min=0, max=65535),
            cv.Optional(CONF_VIRTUAL_LENGTH): cv.int_range(min=0, max=65535),
            # where the receiver sits (CONFIG 140; one setting for the receiver, any one of its lights may say it, the others
            # leave it out or agree): chain = the last board of a normal Ultra line (no digital: on the data line), stream =
            # in a pixel strip's data path (digital: on the data line), auto (the default) = the line decides
            cv.Optional(CONF_MODE): cv.one_of(*LINE_MODES, lower=True),
            # standalone: what the receiver does without the controller (stored with the light's state)
            cv.Optional(CONF_STANDALONE): cv.Schema(
                {
                    cv.Optional(CONF_POWER_ON, default=False): cv.boolean,
                    cv.Optional(CONF_ON_SIGNAL_LOSS, default=False): cv.boolean,
                    cv.Optional(CONF_STREAM_WINS, default=False): cv.boolean,
                    cv.Optional(CONF_SIGNAL_LOSS_TIMEOUT, default="3s"): cv.All(
                        cv.positive_time_period_milliseconds,
                        cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(seconds=60)),
                    ),
                }
            ),
            # store this light's effect on the receiver this long after the last change (its power-on effect)
            cv.Optional(CONF_STORE_AFTER): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=5), max=cv.TimePeriod(hours=1)),
            ),
            # On a Diff-Power port: the port stays powered this long after this light went off
            cv.Optional(CONF_STANDBY_AFTER): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(hours=24)),
            ),
            # Lights that stream pixels to the same strip (an ESPHome addressable light, WLED): switching one on
            # switches the other off, and this light hands the strip back to the stream first
            cv.Optional(CONF_EXCLUSIVE_WITH): cv.ensure_list(cv.use_id(light.LightState)),
            # the colours c2 and c3 of effects that use them, and the sliders' starting values
            cv.Optional(CONF_COLOR_2, default=0x000000): rgb_value,
            cv.Optional(CONF_COLOR_3, default=0x000000): rgb_value,
            cv.Optional(CONF_SPEED, default=128): cv.int_range(min=0, max=255),
            cv.Optional(CONF_INTENSITY, default=128): cv.int_range(min=0, max=255),
            cv.Optional(CONF_PALETTE, default=0): palette_index,
            cv.Optional(CONF_CUSTOM1, default=128): cv.int_range(min=0, max=255),
            cv.Optional(CONF_CUSTOM2, default=128): cv.int_range(min=0, max=255),
            # the entities: Speed, Intensity and Palette of the light (controls: false leaves them out), Custom 1 / 2
            cv.Optional(CONF_CONTROLS, default=True): cv.boolean,
            cv.Optional(CONF_CUSTOM_CONTROLS, default=False): cv.boolean,
            # what the receiver reports (page 0x81, with the line's telemetry): mode, effect, frame rate, load; on a normal
            # Ultra line (chain mode) also its line (Line status, once per receiver)
            cv.Optional(CONF_STATUS, default=True): cv.boolean,
            cv.Optional(CONF_CATALOGUE_EFFECTS, default=True): cv.boolean,
        }
    ),
    _validate_range,
)


def _pixel_lights(full_config) -> list:
    return [c for c in full_config.get("light", []) or [] if c.get("platform") == "ultra_digital"]


def _final_validate(config: ConfigType) -> ConfigType:
    full = fv.full_config.get()
    hub = next((h for h in full.get("dig2analog_ultra", []) or [] if h[CONF_ID] == config[CONF_DIG2ANALOG_ULTRA_ID]), None)
    if hub is None:
        raise cv.Invalid("the data line (dig2analog_ultra) is missing", [CONF_DIG2ANALOG_ULTRA_ID])
    digital = "digital" in hub
    if config[CONF_BOARD] >= hub[CONF_NUM_BOARDS]:
        raise cv.Invalid(
            f"board {config[CONF_BOARD]} out of range, '{hub[CONF_ID]}' has num_boards: {hub[CONF_NUM_BOARDS]} "
            "(boards are 0-based)",
            [CONF_BOARD],
        )
    mine = (str(config[CONF_DIG2ANALOG_ULTRA_ID]), config[CONF_BOARD])
    mode = _receiver_mode(config, [c for c in _pixel_lights(full) if c[CONF_ID] != config[CONF_ID]
                                   and (str(c[CONF_DIG2ANALOG_ULTRA_ID]), c[CONF_BOARD]) == mine])
    if digital and mode == "chain":
        raise cv.Invalid(
            f"mode: chain is for a normal Ultra line: '{hub[CONF_ID]}' is a digital port (digital:), where the receiver "
            "sits in a pixel strip's data path (mode: stream, or auto)",
            [CONF_MODE],
        )
    if not digital and mode == "stream":
        raise cv.Invalid(
            f"mode: stream needs a digital port: '{hub[CONF_ID]}' is a normal Ultra line. A receiver in a pixel strip's "
            "data path needs its frames behind markers (give the data line digital: and num_boards: 1); on a normal "
            "Ultra line the receiver is its last board (mode: chain, or auto)",
            [CONF_MODE],
        )
    if not digital:
        # a chain end: nothing is forwarded (protocol 0.2.9, phase 1), so the receiver is the last board of its line, and
        # the only Ultra-Digital receiver on it
        last = hub[CONF_NUM_BOARDS] - 1
        if config[CONF_BOARD] != last:
            others = [c for c in _pixel_lights(full) if str(c[CONF_DIG2ANALOG_ULTRA_ID]) == str(hub[CONF_ID])
                      and c[CONF_BOARD] != config[CONF_BOARD]]
            raise cv.Invalid(
                f"board {config[CONF_BOARD]} of '{hub[CONF_ID]}' (num_boards: {hub[CONF_NUM_BOARDS]}): an Ultra-Digital "
                f"receiver on a normal Ultra line is its last board, board: {last} (it forwards nothing, so a board behind "
                "it would not be reached)"
                + (", and there is only one on a line" if others else ""),
                [CONF_BOARD],
            )
        if config.get(CONF_EXCLUSIVE_WITH):
            raise cv.Invalid(
                "exclusive_with is for a digital port's streaming light: a receiver on a normal Ultra line shows only its "
                "own effects, there is no pixel stream to share the strip with",
                [CONF_EXCLUSIVE_WITH],
            )
    for other in full.get("light", []) or []:
        if other.get("platform") == "dig2analog_ultra" and (
            str(other.get(CONF_DIG2ANALOG_ULTRA_ID)), other.get(CONF_BOARD)
        ) == mine:
            raise cv.Invalid(
                f"board {config[CONF_BOARD]} of '{hub[CONF_ID]}' is a dig2analog_ultra light's: an Ultra-Digital "
                "receiver has pixel lights only",
                [CONF_BOARD],
            )
    # the lights of one receiver: their own segments, one strip
    for other in _pixel_lights(full):
        if other[CONF_ID] == config[CONF_ID]:
            continue
        if (str(other[CONF_DIG2ANALOG_ULTRA_ID]), other[CONF_BOARD]) != mine:
            continue
        if other[CONF_SEGMENT] == config[CONF_SEGMENT]:
            raise cv.Invalid(
                f"segment {config[CONF_SEGMENT]} of board {config[CONF_BOARD]} is used by '{other.get(CONF_NAME)}' too",
                [CONF_SEGMENT],
            )
        # one strip: the chip, its pixels and order every light says; the other settings (limits, white, standalone) any
        # one light of the receiver may say, the others leave out (two that say it must agree)
        for key in (CONF_CHIP, CONF_NUM_LEDS, CONF_COLOR_ORDER, CONF_AUTO_WHITE, CONF_COLD_WHITE_SHARE, CONF_PIXEL_SMOOTHING, CONF_MAX_FPS,
                    CONF_LED_CURRENT, CONF_CURRENT_LIMIT, CONF_VIRTUAL_OFFSET, CONF_VIRTUAL_LENGTH, CONF_STANDALONE,
                    CONF_MODE):
            both = key in (CONF_CHIP, CONF_NUM_LEDS, CONF_COLOR_ORDER) or (key in other and key in config)
            if both and other.get(key) != config.get(key):
                raise cv.Invalid(
                    f"{key} differs from '{other.get(CONF_NAME)}' on the same receiver: it is one strip's setting, "
                    "say it on one light of the receiver, or the same on each",
                    [key],
                )
    for i, eff in enumerate(config.get(CONF_EFFECTS, [])):
        key = next(k for k in eff if k != CONF_TYPE_ID)
        if key.startswith("ultra_") and key != PIXEL_FX_EFFECT:
            raise cv.Invalid(
                f"effect {key} runs on dig2analog_ultra lights; this light's effects are the catalogue's "
                f"({PIXEL_FX_EFFECT}) and ESPHome's own",
                [CONF_EFFECTS, i],
            )
    standalone = config.get(CONF_STANDALONE) or {}
    if not digital and standalone.get(CONF_STREAM_WINS):
        raise cv.Invalid(
            "stream_wins: a pixel stream taking the strip back has no meaning on a normal Ultra line (the receiver shows only "
            "its own effects there)",
            [CONF_STANDALONE, CONF_STREAM_WINS],
        )
    for i, ex in enumerate(config.get(CONF_EXCLUSIVE_WITH, [])):
        if str(ex) == str(config[CONF_ID]):
            raise cv.Invalid("a light cannot be exclusive with itself", [CONF_EXCLUSIVE_WITH, i])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def _receiver_mode(config: ConfigType, siblings: list) -> str:
    """The receiver's line mode (CONFIG 140): the first light of the receiver that says it, else auto. (The lights of one
    receiver that say it agree: _final_validate.)"""
    for c in [config] + siblings:
        if CONF_MODE in c:
            return c[CONF_MODE]
    return "auto"


def _receiver_config(config: ConfigType, siblings: list) -> list:
    """(id, value) of CONFIG 129-139 and 141 for this strip (140, the line mode, goes on its own: it waits for the receiver's
    capabilities). The options of a receiver's lights are one set: the first light that says a setting gives it, else its
    default."""

    def pick(key, default):
        for c in [config] + siblings:
            if key in c:
                return c[key]
        return default

    chip = config[CONF_CHIP]
    text, order = config[CONF_COLOR_ORDER]
    white = AUTO_WHITE[pick(CONF_AUTO_WHITE, "accurate")]
    share = pick(CONF_COLD_WHITE_SHARE, None)
    if share is not None:
        white |= int(round(share * 255)) << 8
    standalone = pick(CONF_STANDALONE, {})
    policy = (
        (PXP_POWER_ON if standalone.get(CONF_POWER_ON) else 0)
        | (PXP_ON_LOSS if standalone.get(CONF_ON_SIGNAL_LOSS) else 0)
        | (PXP_STREAM_WINS if standalone.get(CONF_STREAM_WINS) else 0)
    )
    timeout = standalone.get(CONF_SIGNAL_LOSS_TIMEOUT)
    limit = pick(CONF_CURRENT_LIMIT, 0)
    return [
        (CFG_CHIP, CHIPS[chip][3]),
        (CFG_ORDER, order),
        (CFG_PIXELS, config[CONF_NUM_LEDS]),
        (CFG_FPS, pick(CONF_MAX_FPS, 0)),
        (CFG_MA_CH, int(round(pick(CONF_LED_CURRENT, 0.020) * 10000))),   # 0.1 mA
        (CFG_LIMIT_MA, 0 if limit == 0 else int(round(limit * 1000))),
        (CFG_WHITE, white),
        (CFG_VOFFSET, pick(CONF_VIRTUAL_OFFSET, 0)),
        (CFG_VLENGTH, pick(CONF_VIRTUAL_LENGTH, 0)),
        (CFG_POLICY, policy),
        (CFG_LOSS_TIMEOUT, 30 if timeout is None else int(round(timeout.total_milliseconds / 100))),
        (CFG_SMOOTH, PIXEL_SMOOTHING[pick(CONF_PIXEL_SMOOTHING, "auto")]),
    ]


def _flags(config: ConfigType) -> int:
    return (
        (PXF_REVERSE if config[CONF_REVERSE] else 0)
        | (PXF_MIRROR if config[CONF_MIRROR] else 0)
        | (PXF_LOCK if config[CONF_LOCK] else 0)
    )


def _entity_conf(config: ConfigType, label: str, suffix: str) -> dict:
    """An entity of the light: on the light's device (device_id) it is just `label` (Home Assistant puts the device's
    name in front), else the light's name and the label"""
    base = config.get(CONF_NAME) or ""
    conf = {CONF_ID: f"{config[CONF_ID]}_{suffix}", CONF_NAME: label if CONF_DEVICE_ID in config or not base else f"{base} {label}"}
    if CONF_DEVICE_ID in config:
        conf[CONF_DEVICE_ID] = config[CONF_DEVICE_ID]
    return conf


async def _entity_number(var, config, which: int, label: str, icon: str, default: int):
    conf = _entity_conf(config, label, label.lower().replace(" ", "_"))
    ent = await number.new_number(
        number.number_schema(PixelNumber, icon=icon, entity_category=ENTITY_CATEGORY_CONFIG)(conf),
        min_value=0,
        max_value=255,
        step=1,
    )
    CORE.component_ids.add(conf[CONF_ID])
    await cg.register_component(ent, {})
    cg.add(ent.set_light(var))
    cg.add(ent.set_param(which))


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    hub = await cg.get_variable(config[CONF_DIG2ANALOG_ULTRA_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_board(config[CONF_BOARD]))
    cg.add(hub.set_pixel_board(config[CONF_BOARD]))   # (before the hub's setup: no light indices are cleared on it)
    cg.add(var.set_segment(config[CONF_SEGMENT]))
    cg.add(var.set_range(config[CONF_SEGMENT_START], config[CONF_SEGMENT_LENGTH]))
    cg.add(var.set_flags(_flags(config)))
    # the strip's frame on the wire: a hand-back to the stream waits for the receiver to finish its frame
    bytes_per_led = CHIPS[config[CONF_CHIP]][1]
    bit_ns = 2667 if config[CONF_CHIP] == "ws2811_400k" else 1334   # (the receiver's local frames: 48 samples a bit, 6.7 % longer than 1.25 us)
    cg.add(var.set_strip_frame_us(config[CONF_NUM_LEDS] * bytes_per_led * 8 * bit_ns // 1000))
    siblings = [
        c for c in _pixel_lights(CORE.config)
        if c[CONF_ID] != config[CONF_ID]
        and (str(c[CONF_DIG2ANALOG_ULTRA_ID]), c[CONF_BOARD]) == (str(config[CONF_DIG2ANALOG_ULTRA_ID]), config[CONF_BOARD])
    ]
    for cfg_id, value in _receiver_config(config, siblings):
        cg.add(var.set_receiver_config(cfg_id, cg.RawExpression(value) if isinstance(value, str) else value))
    hub_conf = next(h for h in CORE.config.get("dig2analog_ultra", []) if h[CONF_ID] == config[CONF_DIG2ANALOG_ULTRA_ID])
    digital = "digital" in hub_conf
    # CONFIG 140 as the YAML says it (auto too: a board forced earlier goes back to deciding itself), and the mode the line
    # needs: stream on a digital port, chain on a normal Ultra line
    cg.add(var.set_line_mode(LINE_MODES[_receiver_mode(config, siblings)], LINE_MODES["stream" if digital else "chain"]))
    cat = catalogue.load()
    cg.add(var.set_solid_id(cat.solid_id))
    cg.add(var.set_palette_ids(cat.palette_ids()))
    cg.add(var.set_defaults(config[CONF_SPEED], config[CONF_INTENSITY], config[CONF_PALETTE], config[CONF_CUSTOM1], config[CONF_CUSTOM2]))
    c2, c3 = config[CONF_COLOR_2], config[CONF_COLOR_3]
    cg.add(var.set_colors(c2, c3))
    if CONF_STANDBY_AFTER in config:
        cg.add(var.set_standby_after(int(config[CONF_STANDBY_AFTER].total_milliseconds)))
    if CONF_STORE_AFTER in config:
        cg.add(var.set_store_after(int(config[CONF_STORE_AFTER].total_milliseconds)))
    await light.register_light(var, config)
    await cg.register_component(var, config)
    for eff in config.get(CONF_EFFECTS, []):
        if PIXEL_FX_EFFECT in eff:
            cg.add(var.add_effect(await cg.get_variable(eff[CONF_TYPE_ID])))
    for ex in config.get(CONF_EXCLUSIVE_WITH, []):
        cg.add(var.add_exclusive(await cg.get_variable(ex)))
    if config[CONF_CONTROLS]:
        await _entity_number(var, config, 0, "Speed", "mdi:speedometer", config[CONF_SPEED])
        await _entity_number(var, config, 1, "Intensity", "mdi:gauge", config[CONF_INTENSITY])
        conf = _entity_conf(config, "Palette", "palette")
        ent = await select.new_select(
            select.select_schema(PixelPalette, icon="mdi:palette", entity_category=ENTITY_CATEGORY_CONFIG)(conf),
            options=cat.palette_options(),
        )
        CORE.component_ids.add(conf[CONF_ID])
        await cg.register_component(ent, {})
        cg.add(ent.set_light(var))
    if config[CONF_CUSTOM_CONTROLS]:
        await _entity_number(var, config, 2, "Custom 1", "mdi:tune", config[CONF_CUSTOM1])
        await _entity_number(var, config, 3, "Custom 2", "mdi:tune", config[CONF_CUSTOM2])
    if config[CONF_STATUS] and hub_conf.get(CONF_TELEMETRY):
        conf = _entity_conf(config, "Local effect status", "status")
        ts = await text_sensor.new_text_sensor(
            text_sensor.text_sensor_schema(icon="mdi:led-strip-variant", entity_category=ENTITY_CATEGORY_DIAGNOSTIC)(conf)
        )
        CORE.component_ids.add(conf[CONF_ID])
        cg.add(var.set_status_sensor(ts))
        # the receiver's line (chain mode): once per receiver, on the light with its lowest segment
        if not digital and all(config[CONF_SEGMENT] < c[CONF_SEGMENT] for c in siblings):
            conf = _entity_conf(config, "Line status", "line")
            ts = await text_sensor.new_text_sensor(
                text_sensor.text_sensor_schema(icon="mdi:lan-connect", entity_category=ENTITY_CATEGORY_DIAGNOSTIC)(conf)
            )
            CORE.component_ids.add(conf[CONF_ID])
            cg.add(var.set_line_sensor(ts))
