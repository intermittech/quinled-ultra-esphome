"""The analog lights (light platforms dig2analog_plus and ur_analog_m: one schema, one implementation): 1-5
channels of one board, board gamma/transitions and host effects. Not a platform itself."""

import json

import esphome.codegen as cg
from esphome.components import light
from esphome.components.light.effects import (
    MONOCHROMATIC_EFFECTS,
    RGB_EFFECTS,
    register_monochromatic_effect,
    register_rgb_effect,
)
from esphome.components.light.types import (
    COLOR_MODES,
    ColorMode,
    LightColorValues,
    LightEffect,
    StrobeLightEffectColor,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_ALPHA,
    CONF_BLUE,
    CONF_BRIGHTNESS,
    CONF_CHANNEL,
    CONF_COLD_WHITE,
    CONF_COLD_WHITE_COLOR_TEMPERATURE,
    CONF_COLOR_BRIGHTNESS,
    CONF_COLOR_INTERLOCK,
    CONF_COLOR_TEMPERATURE,
    CONF_COLOR_MODE,
    CONF_COLORS,
    CONF_CONSTANT_BRIGHTNESS,
    CONF_DURATION,
    CONF_EFFECTS,
    CONF_GREEN,
    CONF_ID,
    CONF_INTENSITY,
    CONF_MAX_BRIGHTNESS,
    CONF_MAX_POWER,
    CONF_MIN_BRIGHTNESS,
    CONF_MIN_POWER,
    CONF_NAME,
    CONF_OUTPUT_ID,
    CONF_RED,
    CONF_RESTORE_MODE,
    CONF_SPEED,
    CONF_STATE,
    CONF_TRANSITION_LENGTH,
    CONF_TYPE_ID,
    CONF_UPDATE_INTERVAL,
    CONF_WARM_WHITE,
    CONF_WARM_WHITE_COLOR_TEMPERATURE,
    CONF_WHITE,
)
from esphome.core import CORE
from esphome.types import ConfigType

from .board_position import board_position, board_slot
from .receiver_schema import ANALOG_LIGHT_PLATFORMS, OUTPUT_MODEL_SCHEMA, light_output_model_to_code, light_telemetry_to_code
from . import (
    CHANNEL_KEYS,
    CONF_ULTRA_LINE_ID,
    EFFECT_GLIDE_MODES,
    HARD_EFFECTS,
    ROLE_KEYS,
    UltraHub,
    _channel_map,
    ultra_line_ns,
    light_roles,
    validate_channel,
    validate_effect_glide,
)


UltraLight = ultra_line_ns.class_("UltraLight", light.LightOutput, cg.Component)

CONF_BOARD = "board"
CONF_ON_LENGTH = "on_length"
CONF_OFF_LENGTH = "off_length"
CONF_PERIOD = "period"
CONF_SATURATION = "saturation"
CONF_ZERO_MEANS_ZERO = "zero_means_zero"
CONF_POWER_ON_SCENE = "power_on_scene"
CONF_STANDBY_AFTER = "standby_after"
CONF_POWER_ON_LEVEL = "power_on_level"
CONF_SIGNAL_LOSS_LEVEL = "signal_loss_level"
CONF_FADE = "fade"
CONF_DIM_TO_WARM = "dim_to_warm"
CONF_RED_ASSIST = "red_assist"
CONF_WARM_COLOR_TEMPERATURE = "warm_color_temperature"
CONF_POINTS = "points"
DTW_POINTS_MAX = 4  # ULTRA_OPT_POINTS_MAX: what fits the LIGHT_OPT slot
CONF_EFFECT_GLIDE = "effect_glide"
# Hard steps bypass glide; all effects run on the host.
EFFECT_HARD = 1

# restore_mode -> what the board shows at its power-up (UltraLight PowerOnScene)
POWER_ON_SCENE_MODES = {
    "ALWAYS_ON": 1,
    "RESTORE_DEFAULT_OFF": 2,
    "RESTORE_DEFAULT_ON": 2,
    "RESTORE_AND_ON": 3,
    "RESTORE_INVERTED_DEFAULT_OFF": 4,
    "RESTORE_INVERTED_DEFAULT_ON": 4,
}

# channel combinations -> ESPHome light type (rgbww, rgbw, rgb, cwww, monochromatic)
_TYPES = {
    frozenset({"channel"}): "monochromatic",
    frozenset({"white"}): "monochromatic",
    frozenset({"cold_white", "warm_white"}): "cwww",
    frozenset({"red", "green", "blue"}): "rgb",
    frozenset({"red", "green", "blue", "white"}): "rgbw",
    frozenset({"red", "green", "blue", "cold_white", "warm_white"}): "rgbww",
}

_POWER = cv.All(cv.percentage)

DIM_TO_WARM_POINT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_BRIGHTNESS): cv.percentage,
        cv.Required(CONF_COLOR_TEMPERATURE): cv.color_temperature,
    }
)

DIM_TO_WARM_SCHEMA = cv.Schema(
    {
        # colour temperature at zero brightness (default: the warm white LED's)
        cv.Optional(CONF_WARM_COLOR_TEMPERATURE): cv.color_temperature,
        # or set points: brightness -> colour temperature, interpolated in mireds (the light's own colour temperature
        # at 100 % unless a point is there; below the lowest point its temperature)
        cv.Optional(CONF_POINTS): cv.All(cv.ensure_list(DIM_TO_WARM_POINT_SCHEMA), cv.Length(min=1, max=DTW_POINTS_MAX)),
        # RGBCCT: red (+ a quarter of it on green) below the WW LED's temperature, as a share of the WW level
        cv.Optional(CONF_RED_ASSIST, default="0%"): cv.percentage,
    }
)


def _validate_dim_to_warm(value):
    """true / false, or the options"""
    if isinstance(value, bool) or not isinstance(value, dict):
        return DIM_TO_WARM_SCHEMA({}) if cv.boolean(value) else None
    return DIM_TO_WARM_SCHEMA(value)


_LEVEL_RGB = (CONF_RED, CONF_GREEN, CONF_BLUE)
_LEVEL_CWW = (CONF_COLOR_TEMPERATURE, CONF_COLD_WHITE, CONF_WARM_WHITE)

STORED_LEVEL_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_STATE, default=True): cv.boolean,
        cv.Optional(CONF_FADE, default="300ms"): cv.All(cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(seconds=1))),
        # the values it runs on (the board's power-on scene); colours not given are 0, all 100 % when none is given
        cv.Optional(CONF_BRIGHTNESS, default=1.0): cv.percentage,
        cv.Optional(CONF_COLOR_TEMPERATURE): cv.color_temperature,
        cv.Optional(CONF_RED): cv.percentage,
        cv.Optional(CONF_GREEN): cv.percentage,
        cv.Optional(CONF_BLUE): cv.percentage,
        cv.Optional(CONF_WHITE): cv.percentage,
        cv.Optional(CONF_COLD_WHITE): cv.percentage,
        cv.Optional(CONF_WARM_WHITE): cv.percentage,
    }
)


def _validate_stored_level(value):
    if not isinstance(value, dict):
        value = {CONF_BRIGHTNESS: cv.percentage(value)}
    return STORED_LEVEL_SCHEMA(value)


def _effect_key(effect: ConfigType) -> str:
    return next(k for k in effect if k != CONF_TYPE_ID)


def _check_stored_level(config: ConfigType, kind: str, key: str) -> None:
    level = config[key]
    rgb = any(k in level for k in _LEVEL_RGB)
    cww = any(k in level for k in _LEVEL_CWW)
    for keys, ok, what in (
        (_LEVEL_RGB, kind in ("rgb", "rgbw", "rgbww"), "red, green and blue channels"),
        ((CONF_WHITE,), kind == "rgbw", "an RGBW light (a white-only light takes brightness)"),
        (_LEVEL_CWW, kind in ("cwww", "rgbww"), "cold_white and warm_white channels"),
    ):
        for k in keys:
            if k in level and not ok:
                raise cv.Invalid(f"{key}: {k} needs {what}", [key, k])
    if CONF_COLOR_TEMPERATURE in level:
        path = [key, CONF_COLOR_TEMPERATURE]
        if CONF_COLD_WHITE in level or CONF_WARM_WHITE in level:
            raise cv.Invalid(f"{key}: give color_temperature or cold_white / warm_white, not both", path)
        if CONF_COLD_WHITE_COLOR_TEMPERATURE not in config:
            raise cv.Invalid(
                f"{key}: color_temperature needs the light's cold_white_color_temperature and "
                "warm_white_color_temperature",
                path,
            )
        lo, hi = config[CONF_COLD_WHITE_COLOR_TEMPERATURE], config[CONF_WARM_WHITE_COLOR_TEMPERATURE]
        if not lo <= level[CONF_COLOR_TEMPERATURE] <= hi:
            raise cv.Invalid(
                f"{key}: color_temperature must be between the LEDs' {round(1e6 / hi)} K and "
                f"{round(1e6 / lo)} K",
                path,
            )
    if config[CONF_COLOR_INTERLOCK] and rgb and (cww or CONF_WHITE in level):
        raise cv.Invalid(
            f"{key}: with color_interlock give either the RGB colour or the white values", [key]
        )


def _power_on_values(config: ConfigType, kind: str) -> LightColorValues:
    """A stored level as ESPHome light values."""
    level = config[CONF_POWER_ON_LEVEL]
    rgb = any(k in level for k in _LEVEL_RGB)
    white = CONF_WHITE in level
    cww = any(k in level for k in _LEVEL_CWW)
    interlock = config[CONF_COLOR_INTERLOCK]
    mode = {
        "monochromatic": ColorMode.BRIGHTNESS,
        "cwww": ColorMode.COLD_WARM_WHITE,
        "rgb": ColorMode.RGB,
        "rgbw": (ColorMode.RGB if rgb else ColorMode.WHITE) if interlock else ColorMode.RGB_WHITE,
        "rgbww": (ColorMode.RGB if rgb else ColorMode.COLD_WARM_WHITE) if interlock else ColorMode.RGB_COLD_WARM_WHITE,
    }[kind]
    unset = 0.0 if rgb or white or cww else 1.0  # no colour given: ESPHome's defaults (all on)
    return LightColorValues(
        mode,
        level[CONF_STATE],
        level[CONF_BRIGHTNESS],
        1.0,
        *(level.get(k, unset) for k in _LEVEL_RGB),
        level.get(CONF_WHITE, unset),
        level.get(CONF_COLOR_TEMPERATURE, 0.0),  # cold / warm white from it at boot (the light's gamma)
        level.get(CONF_COLD_WHITE, unset),
        level.get(CONF_WARM_WHITE, unset),
    )


def _validate_light(config: ConfigType) -> ConfigType:
    keys = frozenset(k for k in ROLE_KEYS if k in config)
    kind = _TYPES.get(keys)
    if kind is None:
        raise cv.Invalid(
            "give the channels of one light type: channel (monochromatic), cold_white + warm_white, "
            "red + green + blue, + white (rgbw), or + cold_white + warm_white (rgbww)"
        )
    chans = [config[k] for k in ROLE_KEYS if k in config]
    if len(set(chans)) != len(chans):
        raise cv.Invalid("each channel can only be used once")
    cww = kind in ("cwww", "rgbww")
    if CONF_CONSTANT_BRIGHTNESS in config and config[CONF_CONSTANT_BRIGHTNESS] and not cww:
        raise cv.Invalid("constant_brightness needs cold_white and warm_white", [CONF_CONSTANT_BRIGHTNESS])
    if config.get(CONF_COLOR_INTERLOCK) and kind not in ("rgbw", "rgbww"):
        raise cv.Invalid("color_interlock needs RGB and white channels", [CONF_COLOR_INTERLOCK])
    has_ct = [k in config for k in (CONF_COLD_WHITE_COLOR_TEMPERATURE, CONF_WARM_WHITE_COLOR_TEMPERATURE)]
    if any(has_ct) and not cww:
        raise cv.Invalid("color temperatures need cold_white and warm_white")
    if any(has_ct) and not all(has_ct):
        raise cv.Invalid("give both cold_white_color_temperature and warm_white_color_temperature")
    if kind not in ("rgb", "rgbw", "rgbww"):
        for i, eff in enumerate(config.get(CONF_EFFECTS, [])):
            key = next(iter(eff))
            if key in RGB_EFFECTS and key not in MONOCHROMATIC_EFFECTS:
                raise cv.Invalid(f"effect {key} needs an RGB light", [CONF_EFFECTS, i])
    for key in (CONF_MIN_POWER, CONF_MAX_POWER):
        for ch, v in config[key].items():
            if v < 0 or v > 1:
                raise cv.Invalid(f"{key} must be 0-100 %", [key, ch])
    for ch in CHANNEL_KEYS:
        if config[CONF_MIN_POWER][ch] > config[CONF_MAX_POWER][ch]:
            raise cv.Invalid(f"min_power above max_power on {ch}")
    dtw = config.get(CONF_DIM_TO_WARM)
    if dtw is not None:
        if not cww or not all(has_ct):
            raise cv.Invalid(
                "dim_to_warm needs cold_white and warm_white with their colour temperatures", [CONF_DIM_TO_WARM]
            )
        points = dtw.get(CONF_POINTS)
        if points is not None:
            if CONF_WARM_COLOR_TEMPERATURE in dtw:
                raise cv.Invalid(
                    "dim_to_warm: give points or warm_color_temperature, not both", [CONF_DIM_TO_WARM, CONF_POINTS]
                )
            seen = set()
            for i, pt in enumerate(points):
                b16 = round(pt[CONF_BRIGHTNESS] * 65535)
                if b16 < 1:
                    raise cv.Invalid("dim_to_warm: a point's brightness must be above 0 %",
                                     [CONF_DIM_TO_WARM, CONF_POINTS, i, CONF_BRIGHTNESS])
                if b16 in seen:
                    raise cv.Invalid("dim_to_warm: two points with the same brightness",
                                     [CONF_DIM_TO_WARM, CONF_POINTS, i, CONF_BRIGHTNESS])
                seen.add(b16)
                if pt[CONF_COLOR_TEMPERATURE] < config[CONF_COLD_WHITE_COLOR_TEMPERATURE]:
                    raise cv.Invalid("dim_to_warm: a point colder than the cold white LED",
                                     [CONF_DIM_TO_WARM, CONF_POINTS, i, CONF_COLOR_TEMPERATURE])
                if pt[CONF_COLOR_TEMPERATURE] > 1000:
                    raise cv.Invalid("dim_to_warm: a point below 1000 K",
                                     [CONF_DIM_TO_WARM, CONF_POINTS, i, CONF_COLOR_TEMPERATURE])
                if pt[CONF_COLOR_TEMPERATURE] > config[CONF_WARM_WHITE_COLOR_TEMPERATURE] and dtw[CONF_RED_ASSIST] <= 0:
                    raise cv.Invalid(
                        "dim_to_warm: a point warmer than the warm white LED needs red_assist (an RGBCCT light)",
                        [CONF_DIM_TO_WARM, CONF_POINTS, i, CONF_COLOR_TEMPERATURE],
                    )
            warm = max(pt[CONF_COLOR_TEMPERATURE] for pt in points)
        else:
            warm = dtw.get(CONF_WARM_COLOR_TEMPERATURE, config[CONF_WARM_WHITE_COLOR_TEMPERATURE])
        if points is None and warm <= config[CONF_COLD_WHITE_COLOR_TEMPERATURE]:
            raise cv.Invalid(
                "dim_to_warm: warm_color_temperature must be warmer than cold_white_color_temperature",
                [CONF_DIM_TO_WARM, CONF_WARM_COLOR_TEMPERATURE],
            )
        if dtw[CONF_RED_ASSIST] > 0:
            if kind != "rgbww":
                raise cv.Invalid("dim_to_warm: red_assist needs an RGBCCT light (red, green, blue + whites)",
                                 [CONF_DIM_TO_WARM, CONF_RED_ASSIST])
            if warm <= config[CONF_WARM_WHITE_COLOR_TEMPERATURE]:
                raise cv.Invalid(
                    "dim_to_warm: red_assist only works below the warm white LED, so warm_color_temperature (or a "
                    "point) must be warmer than warm_white_color_temperature",
                    [CONF_DIM_TO_WARM, CONF_RED_ASSIST],
                )
    if config[CONF_POWER_ON_SCENE] and str(config[CONF_RESTORE_MODE]) not in POWER_ON_SCENE_MODES:
        raise cv.Invalid(
            f"power_on_scene shows the state the light comes back in, which with restore_mode "
            f"{config[CONF_RESTORE_MODE]} is always off; use e.g. RESTORE_DEFAULT_OFF (last state) or "
            f"ALWAYS_ON (with initial_state)",
            [CONF_POWER_ON_SCENE],
        )
    for key in (CONF_POWER_ON_LEVEL, CONF_SIGNAL_LOSS_LEVEL):
        if key in config:
            _check_stored_level(config, kind, key)
    if config[CONF_POWER_ON_SCENE] and CONF_POWER_ON_LEVEL in config:
        raise cv.Invalid("give power_on_scene or power_on_level, not both")
    return light.validate_color_temperature_channels(config)


def _output_model_defaults(config):
    if isinstance(config, dict) and isinstance(config.get("output_model"), dict):
        config = dict(config)
        model = dict(config["output_model"])
        label = "Output model" if config.get("device_id") else f"{config.get(CONF_NAME) or config.get(CONF_ID, 'Light')} output model"
        model.setdefault(CONF_NAME, label)
        model.setdefault("disabled_by_default", False)
        if "device_id" in config:
            model.setdefault("device_id", config["device_id"])
        config["output_model"] = model
    return config


CONFIG_SCHEMA = cv.All(
    _output_model_defaults,
    light.RGB_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(UltraLight),
            cv.GenerateID(CONF_ULTRA_LINE_ID): cv.use_id(UltraHub),
            # position in the chain, 1 = the first board after the controller (counted like num_boards)
            cv.Optional(CONF_BOARD, default=1): board_position,
            # channels: 1-5 (board terminals CH1..CH5), ch1..ch5, r/g/b/w1/w2
            cv.Optional(CONF_CHANNEL): validate_channel,
            cv.Optional(CONF_RED): validate_channel,
            cv.Optional(CONF_GREEN): validate_channel,
            cv.Optional(CONF_BLUE): validate_channel,
            cv.Optional(CONF_WHITE): validate_channel,
            cv.Optional(CONF_COLD_WHITE): validate_channel,
            cv.Optional(CONF_WARM_WHITE): validate_channel,
            cv.Optional(CONF_COLD_WHITE_COLOR_TEMPERATURE): cv.color_temperature,
            cv.Optional(CONF_WARM_WHITE_COLOR_TEMPERATURE): cv.color_temperature,
            cv.Optional(CONF_CONSTANT_BRIGHTNESS, default=False): cv.boolean,
            cv.Optional(CONF_COLOR_INTERLOCK, default=False): cv.boolean,
            cv.Optional("output_model"): OUTPUT_MODEL_SCHEMA,
            # like the min_power / max_power / zero_means_zero of an ESPHome output; one value or per channel
            cv.Optional(CONF_MIN_POWER, default=0.0): _channel_map(_POWER, "min_power"),
            cv.Optional(CONF_MAX_POWER, default=1.0): _channel_map(_POWER, "max_power"),
            cv.Optional(CONF_ZERO_MEANS_ZERO, default=False): cv.boolean,
            # The board stores the state this light comes back in (restore_mode / initial_state) and shows it
            # after its own power-up, with a short fade into it; the first SET controls the take-over.
            cv.Optional(CONF_POWER_ON_SCENE, default=False): cv.boolean,
            # On a Diff-Power port (the hub's port): the port stays powered this long after this light went off
            # (instead of the port's standby_after), so it reacts at once when switched again
            cv.Optional(CONF_STANDBY_AFTER): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(hours=24)),
            ),
            # The two stored output levels, each with a short fade into it.
            cv.Optional(CONF_POWER_ON_LEVEL): _validate_stored_level,
            cv.Optional(CONF_SIGNAL_LOSS_LEVEL): _validate_stored_level,
            # Brightness drives the colour temperature on the board (incandescent-like): from the light's own
            # temperature at full brightness to warm_color_temperature at zero, during transitions and effects too.
            cv.Optional(CONF_DIM_TO_WARM): _validate_dim_to_warm,
            # ESPHome's own effects (flicker, lambda, ...): the board glides between their steps (auto: small steps
            # over the step interval, jumps up at once, down with a 50 ms tail; strobe stays hard), true, or false
            cv.Optional("effect_rate"): cv.int_range(min=1, max=20),
            cv.Optional(CONF_EFFECT_GLIDE, default="auto"): validate_effect_glide,
        }
    ),
    _validate_light,
)


def _light_index(config: ConfigType) -> int:
    """Position of this light among the lights of the same board (0..4), in YAML order."""
    idx = 0
    for other in CORE.config.get("light", []) or []:
        if other.get("platform") not in ANALOG_LIGHT_PLATFORMS:
            continue
        if other[CONF_ID] == config[CONF_ID]:
            return idx
        if other.get(CONF_ULTRA_LINE_ID) == config[CONF_ULTRA_LINE_ID] and other.get(
            CONF_BOARD
        ) == config[CONF_BOARD]:
            idx += 1
    return idx


def output_model_text(config):
    """Versioned JSON, physical terminals in logical RGB/CW/WW order; at most 255 ASCII bytes.

    nonlinear flags YAML settings that a scalar HA light state cannot reconstruct (dim-to-warm / output ranges).
    Gamma matches LIGHT_DEF's millistep rounding / clipping, including gamma 0's linear semantics.
    """
    gamma = config.get("gamma_correct", 2.8)
    gamma = 1 if gamma <= 0 else max(1, int(min(65, gamma) * 1000 + 0.5)) / 1000
    model = {
        "v": 1, "channels": [config[k] + 1 for k in ROLE_KEYS if k in config], "gamma_correct": gamma,
        "constant_brightness": config[CONF_CONSTANT_BRIGHTNESS], "color_interlock": config[CONF_COLOR_INTERLOCK],
        "min_mireds": float(f"{config.get(CONF_COLD_WHITE_COLOR_TEMPERATURE, 0):.6g}"),
        "max_mireds": float(f"{config.get(CONF_WARM_WHITE_COLOR_TEMPERATURE, 0):.6g}"),
        "native_type": "ultra",
        "nonlinear": config.get(CONF_DIM_TO_WARM) is not None
                     or any(config[CONF_MIN_POWER][ch] != 0 or config[CONF_MAX_POWER][ch] != 1 for ch in CHANNEL_KEYS),
    }
    return json.dumps(model, separators=(",", ":"), allow_nan=False)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    hub = await cg.get_variable(config[CONF_ULTRA_LINE_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_board(board_slot(config[CONF_BOARD])))
    if "platform" in config:  # (the log of light changes names the board with it)
        cg.add(var.set_platform(config["platform"]))
    index = _light_index(config)
    if index >= 5:
        raise cv.Invalid(f"board {config[CONF_BOARD]} has more than 5 lights")
    cg.add(var.set_light_index(index))
    for channel, role in light_roles(config):
        cg.add(var.set_role(channel, role))
    for idx, ch in enumerate(CHANNEL_KEYS):
        cg.add(var.set_power_range(idx, config[CONF_MIN_POWER][ch], config[CONF_MAX_POWER][ch]))
    cg.add(var.set_zero_means_zero(config[CONF_ZERO_MEANS_ZERO]))
    cg.add(var.set_constant_brightness(config[CONF_CONSTANT_BRIGHTNESS]))
    cg.add(var.set_color_interlock(config[CONF_COLOR_INTERLOCK]))
    if CONF_COLD_WHITE_COLOR_TEMPERATURE in config:
        cg.add(var.set_cold_white_temperature(config[CONF_COLD_WHITE_COLOR_TEMPERATURE]))
        cg.add(var.set_warm_white_temperature(config[CONF_WARM_WHITE_COLOR_TEMPERATURE]))
    if config[CONF_POWER_ON_SCENE]:
        cg.add(var.set_power_on_scene(POWER_ON_SCENE_MODES[str(config[CONF_RESTORE_MODE])]))
    if CONF_STANDBY_AFTER in config:
        cg.add(var.set_standby_after(int(config[CONF_STANDBY_AFTER].total_milliseconds)))
    if (dtw := config.get(CONF_DIM_TO_WARM)) is not None:
        points = dtw.get(CONF_POINTS, [])
        if points:
            warm = max(pt[CONF_COLOR_TEMPERATURE] for pt in points)  # the red assist scale
        else:
            warm = dtw.get(CONF_WARM_COLOR_TEMPERATURE, config[CONF_WARM_WHITE_COLOR_TEMPERATURE])
        cg.add(var.set_dim_to_warm_config(warm, dtw[CONF_RED_ASSIST]))
        for pt in points:
            cg.add(var.add_dim_to_warm_point(pt[CONF_BRIGHTNESS], pt[CONF_COLOR_TEMPERATURE]))
    if "effect_rate" in config:
        cg.add(var.set_effect_rate(config["effect_rate"]))
    glide = config[CONF_EFFECT_GLIDE]
    cg.add(var.set_effect_glide(EFFECT_GLIDE_MODES[glide]))
    for index, effect in enumerate(config.get(CONF_EFFECTS, []), start=1):
        key = _effect_key(effect)
        if key in HARD_EFFECTS or key == "ultra_strobe":
            cg.add(var.set_effect_kind(index, EFFECT_HARD))
    await light.register_light(var, config)
    kind = _TYPES[frozenset(k for k in ROLE_KEYS if k in config)]
    for slot, key in enumerate((CONF_POWER_ON_LEVEL, CONF_SIGNAL_LOSS_LEVEL)):
        if key in config:
            level = config[key]
            cg.add(var.set_stored_level(slot, _power_on_values({**config, CONF_POWER_ON_LEVEL: level}, kind),
                                       round(level[CONF_FADE].total_milliseconds / 100)))
    # a light on a Diff-Power receiver with several lights: its estimated power / energy and status
    await light_telemetry_to_code(var, config, sum(1 << config[k] for k in ROLE_KEYS if k in config))
    await light_output_model_to_code(config, output_model_text(config))


# ------------------------------------------------------------------------------------------- effects (on the hub)
UltraPulseEffect = ultra_line_ns.class_("UltraPulseEffect", LightEffect)
UltraStrobeEffect = ultra_line_ns.class_("UltraStrobeEffect", LightEffect)
UltraFlickerEffect = ultra_line_ns.class_("UltraFlickerEffect", LightEffect)
UltraRandomEffect = ultra_line_ns.class_("UltraRandomEffect", LightEffect)
UltraCandleEffect = ultra_line_ns.class_("UltraCandleEffect", LightEffect)
UltraBreatheEffect = ultra_line_ns.class_("UltraBreatheEffect", LightEffect)
UltraColorLoopEffect = ultra_line_ns.class_("UltraColorLoopEffect", LightEffect)
UltraSunEffect = ultra_line_ns.class_("UltraSunEffect", LightEffect)

_MS = cv.positive_time_period_milliseconds


def _max_ms(limit_ms):
    return cv.All(_MS, cv.Range(max=cv.TimePeriod(milliseconds=limit_ms)))


@register_monochromatic_effect(
    "ultra_pulse",
    UltraPulseEffect,
    "Pulse",
    {
        cv.Optional(CONF_TRANSITION_LENGTH, default="1s"): cv.Any(
            _max_ms(0xFFFFFF),
            cv.Schema(
                {
                    cv.Required(CONF_ON_LENGTH): _max_ms(0xFFFFFF),
                    cv.Required(CONF_OFF_LENGTH): _max_ms(0xFFFFFF),
                }
            ),
        ),
        cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.All(_MS, cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(milliseconds=0xFFFFFF))),
        cv.Optional(CONF_MIN_BRIGHTNESS, default="0%"): cv.percentage,
        cv.Optional(CONF_MAX_BRIGHTNESS, default="100%"): cv.percentage,
    },
)
async def ultra_pulse_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    length = config[CONF_TRANSITION_LENGTH]
    if isinstance(length, dict):
        cg.add(var.set_transition_on_length(length[CONF_ON_LENGTH]))
        cg.add(var.set_transition_off_length(length[CONF_OFF_LENGTH]))
    else:
        cg.add(var.set_transition_on_length(length))
        cg.add(var.set_transition_off_length(length))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_min_max_brightness(config[CONF_MIN_BRIGHTNESS], config[CONF_MAX_BRIGHTNESS]))
    return var


_STROBE_COLOR = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_STATE, default=True): cv.boolean,
            cv.Optional(CONF_BRIGHTNESS, default=1.0): cv.percentage,
            cv.Optional(CONF_COLOR_MODE): cv.enum(COLOR_MODES, upper=True, space="_"),
            cv.Optional(CONF_COLOR_BRIGHTNESS, default=1.0): cv.percentage,
            cv.Optional(CONF_RED, default=1.0): cv.percentage,
            cv.Optional(CONF_GREEN, default=1.0): cv.percentage,
            cv.Optional(CONF_BLUE, default=1.0): cv.percentage,
            cv.Optional(CONF_WHITE, default=1.0): cv.percentage,
            cv.Optional(CONF_COLD_WHITE, default=1.0): cv.percentage,
            cv.Optional(CONF_WARM_WHITE, default=1.0): cv.percentage,
            cv.Required(CONF_DURATION): _max_ms(65535),
            cv.Optional(CONF_TRANSITION_LENGTH, default="0s"): _max_ms(65535),
        }
    ),
)


@register_monochromatic_effect(
    "ultra_strobe",
    UltraStrobeEffect,
    "Strobe",
    {
        cv.Optional(
            CONF_COLORS,
            default=[
                {CONF_STATE: True, CONF_DURATION: "0.5s"},
                {CONF_STATE: False, CONF_DURATION: "0.5s"},
            ],
        ): cv.All(cv.ensure_list(_STROBE_COLOR), cv.Length(min=2, max=8)),
    },
)
async def ultra_strobe_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    colors = [
        cg.StructInitializer(
            StrobeLightEffectColor,
            (
                "color",
                LightColorValues(
                    color.get(CONF_COLOR_MODE, ColorMode.UNKNOWN),
                    color[CONF_STATE],
                    color[CONF_BRIGHTNESS],
                    color[CONF_COLOR_BRIGHTNESS],
                    color[CONF_RED],
                    color[CONF_GREEN],
                    color[CONF_BLUE],
                    color[CONF_WHITE],
                    0.0,
                    color[CONF_COLD_WHITE],
                    color[CONF_WARM_WHITE],
                ),
            ),
            ("duration", color[CONF_DURATION]),
            ("transition_length", color[CONF_TRANSITION_LENGTH]),
        )
        for color in config[CONF_COLORS]
    ]
    cg.add(var.set_colors(colors))
    return var


@register_monochromatic_effect(
    "ultra_flicker",
    UltraFlickerEffect,
    "Flicker",
    {
        cv.Optional(CONF_ALPHA, default=0.95): cv.percentage,
        cv.Optional(CONF_INTENSITY, default=0.015): cv.percentage,
        cv.Optional(CONF_UPDATE_INTERVAL, default="16ms"): cv.All(
            _MS, cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(milliseconds=65535))
        ),
    },
)
async def ultra_flicker_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(var.set_alpha(config[CONF_ALPHA]))
    cg.add(var.set_intensity(config[CONF_INTENSITY]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    return var


@register_monochromatic_effect(
    "ultra_random",
    UltraRandomEffect,
    "Random",
    {
        cv.Optional(CONF_TRANSITION_LENGTH, default="7.5s"): _max_ms(0xFFFFFF),
        cv.Optional(CONF_UPDATE_INTERVAL, default="10s"): cv.All(
            _MS, cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(milliseconds=0xFFFFFF))
        ),
    },
)
async def ultra_random_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(var.set_transition_length(config[CONF_TRANSITION_LENGTH]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    return var


@register_monochromatic_effect(
    "ultra_candle",
    UltraCandleEffect,
    "Candle",
    {
        cv.Optional(CONF_INTENSITY, default="40%"): cv.percentage,
        # 100 % = a natural flame; 50 % calmer, 200 % windy
        cv.Optional(CONF_SPEED, default="100%"): cv.All(cv.possibly_negative_percentage, cv.Range(min=0.1, max=10.0)),
    },
)
async def ultra_candle_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(var.set_intensity(config[CONF_INTENSITY]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var


@register_monochromatic_effect(
    "ultra_breathe",
    UltraBreatheEffect,
    "Breathe",
    {
        cv.Optional(CONF_PERIOD, default="4s"): cv.All(
            _MS, cv.Range(min=cv.TimePeriod(milliseconds=100), max=cv.TimePeriod(milliseconds=0xFFFFFF))
        ),
        cv.Optional(CONF_MIN_BRIGHTNESS, default="0%"): cv.percentage,
        cv.Optional(CONF_MAX_BRIGHTNESS, default="100%"): cv.percentage,
    },
)
async def ultra_breathe_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(var.set_period(config[CONF_PERIOD]))
    cg.add(var.set_min_max_brightness(config[CONF_MIN_BRIGHTNESS], config[CONF_MAX_BRIGHTNESS]))
    return var


@register_rgb_effect(
    "ultra_color_loop",
    UltraColorLoopEffect,
    "Color Loop",
    {
        cv.Optional(CONF_PERIOD, default="30s"): cv.All(
            _MS, cv.Range(min=cv.TimePeriod(milliseconds=100), max=cv.TimePeriod(milliseconds=0xFFFFFF))
        ),
        cv.Optional(CONF_SATURATION, default="100%"): cv.percentage,
    },
)
async def ultra_color_loop_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(var.set_period(config[CONF_PERIOD]))
    cg.add(var.set_saturation(config[CONF_SATURATION]))
    return var


_SUN_SCHEMA = {
    cv.Optional(CONF_DURATION, default="15min"): cv.All(
        _MS, cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(milliseconds=0xFFFFFF))
    ),
}


@register_monochromatic_effect("ultra_sunrise", UltraSunEffect, "Sunrise", _SUN_SCHEMA)
async def ultra_sunrise_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME], True)
    cg.add(var.set_duration(config[CONF_DURATION]))
    return var


@register_monochromatic_effect("ultra_sunset", UltraSunEffect, "Sunset", _SUN_SCHEMA)
async def ultra_sunset_to_code(config, effect_id):
    var = cg.new_Pvariable(effect_id, config[CONF_NAME], False)
    cg.add(var.set_duration(config[CONF_DURATION]))
    return var
