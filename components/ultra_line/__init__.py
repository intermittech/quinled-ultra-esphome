"""QuinLED Dig2Analog-Plus-5ch boards in Ultra mode: the boards run transitions and gamma; analog effects run on the hub.

The hub sends commands on one data line (WS2915-style one-wire pulses, ESP-IDF RMT; or a lane of a
ultra_parlio bus on an ESP32-P4). Lights are the `ultra_line` light platform (light.py). Board
settings are host-managed: the values here are sent to every board at boot and repeated, and never stored in the
boards' flash (except `boot_protocol`).
"""

import logging

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import esp32, esp32_rmt, power_supply
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import (
    CONF_BLUE,
    CONF_ENABLE_TIME,
    CONF_GREEN,
    CONF_ID,
    CONF_INVERTED,
    CONF_KEEP_ON_TIME,
    CONF_NUMBER,
    CONF_PIN,
    CONF_POWER_SUPPLY,
    CONF_RED,
    CONF_RMT_SYMBOLS,
    CONF_USE_DMA,
)
from esphome.core import CORE
from esphome.types import ConfigType

from .board_position import board_position, board_slot, out_of_range, positions_text
from .chain_reset import CHAIN_FIRST_RESET_NS, reset_with_floor_ns
from .parlio_timing import layout as parlio_layout
from .receiver_schema import (
    ANALOG_LIGHT_PLATFORMS,
    CONF_DIRECTION_PIN,
    CONF_TELEMETRY,
    TELEMETRY_SCHEMA,
    telemetry_defaults,
    telemetry_to_code,
    validate_board_platforms,
    validate_telemetry,
    validate_telemetry_lines,
    warn_ram,
)

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["esp32"]
MULTI_CONF = True


def AUTO_LOAD(config=None):
    """The entity types of the Diff-Power receivers' telemetry, only for a line with telemetry: (no config: all)."""
    confs = config if isinstance(config, list) else [config]
    if config is None or any(isinstance(c, dict) and c.get(CONF_TELEMETRY) for c in confs):
        return ["sensor", "binary_sensor", "text_sensor", "button", "number", "select", "switch"]
    return []


ultra_line_ns = cg.esphome_ns.namespace("ultra_line")
UltraHub = ultra_line_ns.class_("UltraHub", cg.Component)
SwitchBoardsToUltraAction = ultra_line_ns.class_("SwitchBoardsToUltraAction", automation.Action)
CalibrateCurrentAction = ultra_line_ns.class_("CalibrateCurrentAction", automation.Action)
CancelCalibrationAction = ultra_line_ns.class_("CancelCalibrationAction", automation.Action)
RetryReceiverAction = ultra_line_ns.class_("RetryReceiverAction", automation.Action)
CommissionAction = ultra_line_ns.class_("CommissionAction", automation.Action)
ShowVersionAction = ultra_line_ns.class_("ShowVersionAction", automation.Action)
# (ultra_parlio's bus, by name: that component imports this one)
UltraParlioBus = ultra_line_ns.class_("UltraParlioBus", cg.Component)

CONF_ULTRA_LINE_ID = "ultra_line_id"
CONF_NUM_BOARDS = "num_boards"
CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_RESET_TIME = "reset_time"
CONF_REFRESH_INTERVAL = "refresh_interval"
CONF_START_DELAY = "start_delay"
CONF_REPEATS = "repeats"
CONF_POWER_ON_DELAY = "power_on_delay"
CONF_PORT = "port"
# the diffpower component's port, by name (no import: a configuration without diffpower must not need it)
DiffPowerPort = cg.esphome_ns.namespace("diffpower").class_("DiffPowerPort", cg.Component)
CONF_PWM_FREQUENCY = "pwm_frequency"
CONF_DITHERING = "dithering"
CONF_DITHER_THRESHOLD = "dither_threshold"
CONF_DIM_FLOOR = "dim_floor"
CONF_MIN_PULSE = "min_pulse"
CONF_PULSE_SKIP_RATE = "pulse_skip_rate"
CONF_STAGGER = "stagger"
CONF_SIGNAL_LOSS = "signal_loss"
CONF_SIGNAL_LOSS_TIMEOUT = "signal_loss_timeout"
CONF_TRIP_RETRIES = "trip_retries"
CONF_TRIP_RETRY_INTERVAL = "trip_retry_interval"
CONF_FEEDBACK_CHANNEL = "feedback_channel"
CONF_POWER_LIMIT = "power_limit"
CONF_CHANNEL_CURRENT = "channel_current"
CONF_MAX_CHANNEL_CURRENT = "max_channel_current"
CONF_MAX_BOARD_CURRENT = "max_board_current"
CONF_WHITE1 = "white1"
CONF_WHITE2 = "white2"
CONF_DEFAULT = "default"
CONF_BOOT_PROTOCOL = "boot_protocol"
CONF_CURRENT_CALIBRATION = "current_calibration"
CONF_ON_BOOT = "on_boot"
CONF_INTERVAL = "interval"
CONF_ONLY_WHEN_OFF = "only_when_off"
CONF_BOARDS = "boards"
CONF_LEVEL = "level"
CONF_PARLIO_ID = "parlio_id"
CONF_DIGITAL = "digital"
CONF_LANE = "lane"
CONF_PORT_TYPE = "port_type"
CONF_EVERY_BOARD_DESCRIPTOR = "every_board_descriptor"

# The physical port (ultra_line_rate.h PortType): the boards forward without redriving, so the boards a port may chain
# and the line speed its board count starts at depend on it: a Diff-Power RS-485 port eight receivers, a
# Dig2Analog-Plus-5ch single-wire data pin fifty boards.
PORT_TYPES = {"diff_power": 1, "d2a_pin": 2}
PORT_MAX_BOARDS = {"diff_power": 8, "d2a_pin": 50}


# light platforms of Diff-Power Ultra Receivers (there are no Diff-Power receivers without the return channel's
# firmware, so one on a line makes it a Diff-Power port)
RECEIVER_PLATFORMS = ("ur_analog_m", "ur_digital_m", "ur_digital_m_stream")


def line_port_type(full, line):
    """(type, marks): the line's port_type; not given: diff_power for a line with a Diff-Power mark, else d2a_pin.
    marks: what makes the line a Diff-Power port (telemetry, a Diff-Power port, a digital port, a receiver light)."""
    marks = [text for key, text in ((CONF_TELEMETRY, "telemetry"), (CONF_PORT, "a Diff-Power port (port:)"),
                                    (CONF_DIGITAL, "a digital port (digital:)")) if key in line]
    marks += [f"a Diff-Power Ultra Receiver light '{c.get('name', c[CONF_ID])}' ({c['platform']})"
              for c in full.get("light", []) or []
              if c.get("platform") in RECEIVER_PLATFORMS and str(c.get(CONF_ULTRA_LINE_ID)) == str(line[CONF_ID])]
    return line.get(CONF_PORT_TYPE) or ("diff_power" if marks else "d2a_pin"), marks


# A Dig2Analog-Plus-5ch pin's speed table: the profile a pin of up to that many boards runs at (the sender's
# ultra_port_line_profile, ultra_sender_update.h ULTRA_D2A_800_MAX_BOARDS / ULTRA_D2A_600_MAX_BOARDS; the schema
# tests check this copy against the vendored header)
D2A_PIN_SPEEDS = ((20, 0), (34, 1), (50, 3))
LINE_RATES = ("800 kHz", "600 kHz", "400 kHz", "200 kHz")


# light platform keys that take a channel, and their roles (ULTRA_ROLE_*)
ROLE_KEYS = ["channel", "red", "green", "blue", "white", "cold_white", "warm_white"]
ROLES = {"channel": 1, "red": 2, "green": 3, "blue": 4, "white": 5, "cold_white": 6, "warm_white": 7}

# Board terminals CH1..CH5 = channel index 0..4 (data order R, G, B, W1, W2 in WS mode).
CHANNEL_KEYS = [CONF_RED, CONF_GREEN, CONF_BLUE, CONF_WHITE1, CONF_WHITE2]
CHANNEL_ALIASES = {key: idx for idx, key in enumerate(CHANNEL_KEYS)}
CHANNEL_ALIASES.update({"r": 0, "g": 1, "b": 2, "w1": 3, "w2": 4})
CHANNEL_ALIASES.update({f"ch{i + 1}": i for i in range(5)})

# ULTRA_CFG_* ids (ultra_protocol.h)
CFG_PWM_PRESET = 1
CFG_PWM_PERIOD = 2
CFG_DITHER = 3
CFG_DITHER_THRESH = 4
CFG_DIM_FLOOR = 5
CFG_MIN_PULSE = 6
CFG_STAGGER = 7
CFG_LOSS = 8
CFG_LOSS_TIMEOUT = 9
CFG_FEEDBACK = 10
CFG_CH_CURRENT = 11
CFG_CH_MAX = 12
CFG_BOARD_MAX = 21
CFG_BOOT_PROTOCOL = 22  # persisted on the board
CFG_RETRY = 24  # persisted on the board (protocol 0.3): count | interval << 8
CFG_SKIP_RATE = 25  # pulse-skip rate, Hz: 2000 / 1000 (boards whose descriptor says SKIP_RATE)

DITHERING = {"on": 0, "off_at_low_levels": 1, "off": 2}
DIM_FLOOR = {"skip": 0, "clamp": 1, "off": 2}
# scene: fade to the stored signal-loss level (unset: dark)
SIGNAL_LOSS = {"hold": 0, "off": 1, "fade_off": 2, "scene": 3}
TRIP_RETRIES_DEFAULT = 10  # the boards' own default: 10 retries 1 s apart
TRIP_RETRIES_FOREVER = 255
FEEDBACK = {"red": 0, "all": 1, "off": 2}
# each mode followed by its spread-spectrum variant; "spread" = adaptive_spread
STAGGER = {"off": 0, "fixed": 1, "fixed_spread": 2, "adaptive": 3, "adaptive_spread": 4, "spread": 4}
BOOT_PROTOCOL = {"auto": 0, "ws2915": 1, "ws2805": 2, "ultra": 3}


def _validate_stagger(value):
    """off / fixed / fixed_spread / adaptive / adaptive_spread (alias spread); true = fixed, false = off."""
    if isinstance(value, bool):
        return "fixed" if value else "off"
    return cv.one_of(*STAGGER, lower=True)(value)


def _validate_trip_retries(value):
    """0 (never retry) to 254, or forever."""
    if isinstance(value, str) and value.strip().lower() in ("forever", "always"):
        return TRIP_RETRIES_FOREVER
    return cv.int_range(min=0, max=254)(value)


def _enum_or_bool(mapping, bools):
    """A mode by name, or YAML's bare `on` / `off` (which YAML reads as booleans) where it is one of the modes:
    `dithering: on`, `signal_loss: off`, `dim_floor: off` work without quotes."""

    def validator(value):
        if isinstance(value, bool):
            if value in bools:
                return bools[value]
            raise cv.Invalid(
                f"'{'on' if value else 'off'}' is not an option here; use one of {', '.join(mapping)} (quote 'on' / "
                f"'off' in YAML, or write it as a word)"
            )
        return cv.enum(mapping, lower=True)(value)

    return validator


def light_roles(config):
    """(channel 0..4, role) of an analog light (dig2analog_plus, ur_analog_m). A light of only `white:` is monochromatic: its channel follows
    the brightness (ESPHome gives a monochromatic light no white value)."""
    keys = [k for k in ROLE_KEYS if k in config]
    return [(config[k], ROLES["channel"] if keys == ["white"] else ROLES[k]) for k in keys]


# effect_glide (lights and strips): how the steps of ESPHome's own effects reach the board. UltraLight EffectGlide.
EFFECT_GLIDE_MODES = {False: 0, True: 1, "auto": 2}
# ESPHome effects whose steps stay hard (commands, no glide) with effect_glide: auto
HARD_EFFECTS = {"strobe"}


def validate_effect_glide(value):
    """auto (small steps glide, jumps up at once, down with a short tail; strobe hard), true (every step glides over
    the step interval) or false (hard steps)."""
    if isinstance(value, str) and value.strip().lower() == "auto":
        return "auto"
    return cv.boolean(value)


def validate_channel(value):
    """1..5 (board terminals CH1..CH5), ch1..ch5, red/green/blue/white1/white2 or r/g/b/w1/w2 -> 0..4."""
    if isinstance(value, int) and not isinstance(value, bool):
        return cv.int_range(min=1, max=5)(value) - 1
    key = cv.string_strict(value).lower()
    if key.isdigit():
        return cv.int_range(min=1, max=5)(int(key)) - 1
    if key not in CHANNEL_ALIASES:
        raise cv.Invalid(f"unknown channel '{value}' (use 1-5, {', '.join(CHANNEL_ALIASES)})")
    return CHANNEL_ALIASES[key]


def _channel_map(value_validator, what, require_all=True):
    """One value for every channel, or a mapping channel -> value (with an optional `default:`)."""

    def validator(value):
        if not isinstance(value, dict):
            v = value_validator(value)
            return {k: v for k in CHANNEL_KEYS}
        out = {}
        default = None
        for raw_key, raw_val in value.items():
            if str(raw_key).lower() == CONF_DEFAULT:
                default = value_validator(raw_val)
                continue
            try:
                name = CHANNEL_KEYS[validate_channel(raw_key)]
            except cv.Invalid as err:
                raise cv.Invalid(str(err), [raw_key]) from err
            if name in out:
                raise cv.Invalid(f"channel '{name}' given twice", [raw_key])
            out[name] = value_validator(raw_val)
        for k in CHANNEL_KEYS:
            if k not in out and default is not None:
                out[k] = default
        missing = [k for k in CHANNEL_KEYS if k not in out]
        if missing and require_all:
            raise cv.Invalid(f"{what}: missing channel(s) {', '.join(missing)} (or give a default:)")
        return {k: out[k] for k in CHANNEL_KEYS if k in out}

    return validator


_PWM_MIN_HZ = 144e6 / 65535
_PWM_MAX_HZ = 100000.0


def _validate_pulse_skip_rate(value):
    hz = cv.frequency(value)
    if round(hz) not in (1000, 2000):
        raise cv.Invalid("pulse_skip_rate must be 2kHz (the default) or 1kHz")
    return round(hz)


def _validate_pwm_frequency(value):
    hz = cv.frequency(value)
    if not _PWM_MIN_HZ * 0.999 <= hz <= _PWM_MAX_HZ * 1.001:
        raise cv.Invalid(f"pwm_frequency must be {_PWM_MIN_HZ:.1f} Hz - 100 kHz")
    return hz


def _validate_reset(value):
    value = cv.positive_time_period_nanoseconds(value)
    us = value.total_nanoseconds / 1000
    if us < 500:
        raise cv.Invalid("reset_time must be at least 500us (Ultra protocol processing gap)")
    if us > 800:
        raise cv.Invalid("reset_time above 800us does not fit one RMT symbol")
    return value


def _validate_digital(value):
    """digital: (or true) -> the options; reset_time: the strip's reset (the latch), the pixel frames' tail and what a
    hand-back from a local effect waits for."""
    if value is None or value is True:
        value = {}
    if value is False:
        raise cv.Invalid("digital: leave it out for a line of Dig2Analog-Plus-5ch boards")
    return DIGITAL_SCHEMA(value)


def _validate_digital_reset(value):
    value = cv.positive_time_period_nanoseconds(value)
    us = value.total_nanoseconds / 1000
    if us < 50 or us > 800:
        raise cv.Invalid("reset_time: 50us to 800us (the strip chip's latch time; WS2812B 280us+, SK6812 80us)")
    return value


DIGITAL_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_RESET_TIME, default="300us"): _validate_digital_reset,
    }
)


def _validate_start_delay(value):
    """auto (default): the hub works out how far ahead commands start; or a fixed time up to 1 s."""
    if isinstance(value, str) and value.strip().lower() == "auto":
        return "auto"
    value = cv.positive_time_period_milliseconds(value)
    if value.total_milliseconds > 1000:
        raise cv.Invalid("start_delay must be auto or at most 1s")
    return value


def _validate_min_pulse(value):
    value = cv.positive_time_period_nanoseconds(value)
    ns = value.total_nanoseconds
    if not 50 <= ns <= 1000:
        raise cv.Invalid("min_pulse must be 50ns - 1us")
    return value


def _validate_dither_threshold(value):
    v = cv.percentage(value)
    if not 0.001 <= v <= 0.255:
        raise cv.Invalid("dither_threshold must be 0.1% - 25.5%")
    return v


POWER_LIMIT_SCHEMA = cv.Schema(
    {
        # LED current of each channel at 100 % (same on every board of this line). Needed for max_channel_current
        # and for boards without current sense; boards with it measure the board current (max_board_current).
        # (both are sent in mA: up to 65.535 A)
        cv.Optional(CONF_CHANNEL_CURRENT): _channel_map(
            cv.All(cv.current, cv.Range(min=0, max=65.535)), "channel_current"
        ),
        # Limit per output channel; only that channel is held back. At most each channel's own ceiling
        # (CHANNEL_CEILINGS: the Dig2Analog-Plus-5ch R / G / B 3 A, W1 / W2 5 A).
        cv.Optional(CONF_MAX_CHANNEL_CURRENT): _channel_map(
            cv.All(cv.current, cv.Range(min=0.01, max=65.535)), "max_channel_current", require_all=False
        ),
        # Limit per board (e.g. below its fuse); all its channels are dimmed together. Boards with the input
        # current sense hold their measured current to it (without channel_current); the others estimate it.
        cv.Optional(CONF_MAX_BOARD_CURRENT): cv.All(cv.current, cv.Range(min=0.01, max=65.0)),
    }
)


# Each channel's own current ceiling (A) on the boards of an analog light platform, independent of the board's own
# limit (the firmware's board.json channel_ma; the board cuts a higher max_channel_current to it): the
# Dig2Analog-Plus-5ch rev 0.4 R / G / B 3 A, W1 / W2 5 A (board 8 A); the Diff-Power Ultra Receiver Analog-Medium
# 3 A or 5 A on every channel by its assembly (5 A: the higher one; a 3 A board takes 3 A).
CHANNEL_CEILINGS = {
    "dig2analog_plus": ("Dig2Analog-Plus-5ch", (3.0, 3.0, 3.0, 5.0, 5.0)),
    "ur_analog_m": ("Diff-Power-UR-Analog-M", (5.0, 5.0, 5.0, 5.0, 5.0)),
}


def validate_channel_ceilings(full, line):
    """max_channel_current per channel at most that channel's own ceiling on every board type of the line"""
    limits = line.get(CONF_POWER_LIMIT, {}).get(CONF_MAX_CHANNEL_CURRENT, {})
    for platform in sorted({light["platform"] for light in _lights_of(full, line[CONF_ID])}):
        if platform not in CHANNEL_CEILINGS:
            continue
        product, ceilings = CHANNEL_CEILINGS[platform]
        for idx, key in enumerate(CHANNEL_KEYS):
            if key in limits and limits[key] > ceilings[idx] + 1e-9:
                raise cv.Invalid(
                    f"power_limit: max_channel_current {key} (CH{idx + 1}) {limits[key]:g} A is above the "
                    f"{product}'s {ceilings[idx]:g} A for that channel (each channel's own ceiling: "
                    + ", ".join(f"CH{i + 1} {c:g} A" for i, c in enumerate(ceilings))
                    + "; the board's own limit is max_board_current)"
                )


def _validate_power_limit(value):
    value = POWER_LIMIT_SCHEMA(value)
    if CONF_MAX_CHANNEL_CURRENT not in value and CONF_MAX_BOARD_CURRENT not in value:
        raise cv.Invalid("power_limit needs max_channel_current and/or max_board_current")
    if CONF_MAX_CHANNEL_CURRENT in value and CONF_CHANNEL_CURRENT not in value:
        raise cv.Invalid("max_channel_current needs channel_current")
    return value


def _validate_cal_level(value):
    """all (the standard test: each channel at 25 / 50 / 75 / 100 %) or one duty 5-100 % -> 0 / 5..100."""
    if isinstance(value, str) and value.strip().lower() == "all":
        return 0
    pct = round(cv.percentage(value) * 100)
    if not 5 <= pct <= 100:
        raise cv.Invalid("level must be all, or 5% - 100%")
    return pct


# Boards with the input current sense: when the hub runs the calibration by itself. Normally it is a manual step (the
# action below), after installing or changing what is connected. A run darkens the pin and tests one board after the
# other (~1.4 s each, visible), so nothing runs unless configured.
CURRENT_CALIBRATION_SCHEMA = cv.Schema(
    {
        # once, 10 s after the ESP boots (with only_when_off: once the lights are off)
        cv.Optional(CONF_ON_BOOT, default=False): cv.boolean,
        # periodically (from boot on); a run that falls due while a light is on waits for them to go off
        cv.Optional(CONF_INTERVAL): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(min=cv.TimePeriod(minutes=10), max=cv.TimePeriod(days=30)),
        ),
        # automatic runs only while every light on this line is off (with a power_supply it is switched on for the
        # run). false: also while they are on (the pin goes dark for the run, then fades back)
        cv.Optional(CONF_ONLY_WHEN_OFF, default=True): cv.boolean,
        # which boards to test (1 = the first board), default all (the whole pin goes dark either way)
        cv.Optional(CONF_BOARDS): cv.ensure_list(board_position),
    }
)

def _validate_transmitter(config):
    """An RMT line (pin) or a lane of a ultra_parlio bus (parlio_id + lane), not both."""
    if (CONF_PIN in config) == (CONF_PARLIO_ID in config):
        raise cv.Invalid("give the line a pin (RMT), or a parlio_id and lane (a ultra_parlio bus)")
    if (CONF_LANE in config) != (CONF_PARLIO_ID in config):
        raise cv.Invalid("lane goes with parlio_id: the bus' pin of that lane is the line's")
    if CONF_PARLIO_ID in config and CONF_USE_DMA in config:
        raise cv.Invalid("use_dma is an RMT option; a PARLIO lane always streams by DMA", [CONF_USE_DMA])
    return config


def _validate_digital_line(config):
    """A digital port: one receiver, an RMT line, and with telemetry the direction pin (the hub lets go of the line)."""
    if CONF_DIGITAL not in config:
        return config
    if config[CONF_NUM_BOARDS] != 1:
        raise cv.Invalid("digital: a digital port has one receiver (num_boards: 1)", [CONF_NUM_BOARDS])
    if CONF_PARLIO_ID in config:
        raise cv.Invalid("digital: needs an RMT line (pin), not a PARLIO lane", [CONF_DIGITAL])
    tel = config.get(CONF_TELEMETRY)
    if tel is not None and CONF_DIRECTION_PIN not in tel:
        raise cv.Invalid(
            "digital with telemetry: give the telemetry a direction_pin (the hub lets go of the data line for the "
            "receiver's answer)",
            [CONF_TELEMETRY],
        )
    return config


CONFIG_SCHEMA = cv.All(
    telemetry_defaults,   # the receivers' entities, named for their devices (before they are validated)
    esp32.only_on_variant(
        unsupported=list(esp32_rmt.VARIANTS_NO_RMT),
        msg_prefix="ultra_line (RMT)",
    ),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraHub),
            # the data line: an RMT channel on this pin, or a lane of a ultra_parlio bus (ESP32-P4)
            cv.Optional(CONF_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PARLIO_ID): cv.use_id(UltraParlioBus),
            cv.Optional(CONF_LANE): cv.int_range(min=0, max=15),
            cv.Required(CONF_NUM_BOARDS): cv.int_range(min=1, max=64),
            # The physical port the line drives (its board limit; the line starts at the speed its board count
            # allows): diff_power (a Diff-Power RS-485 port, at most 8 receivers) or d2a_pin (a Dig2Analog-Plus-5ch
            # data pin, at most 50 boards). Not given: diff_power for a line with telemetry, on a Diff-Power port, a
            # digital port or with a Diff-Power Ultra Receiver light (ur_digital_m, ur_digital_m_stream); otherwise
            # d2a_pin. A data pin can't carry any of those.
            cv.Optional(CONF_PORT_TYPE): cv.one_of(*PORT_TYPES, lower=True),
            # A DIGITAL PORT (protocol 0.3): one Diff-Power Ultra Receiver Digital-Medium (num_boards: 1) in the data
            # path of a pixel strip. The hub sends its frames behind markers in latch gaps (the strip never sees them),
            # the pixel stream of an ur_digital_m_stream light between them, and polls leave the line to the
            # receiver's answer. No preamble, no firmware update over this line.
            cv.Optional(CONF_DIGITAL): _validate_digital,
            # '0' 250 ns, '1' 700 ns: centred on the boards' 472 ns decision, so the pulse-width change of every
            # board's forwarding may add up in either direction over 50 boards (wiki: Ultra line options, Long chains)
            cv.Optional(CONF_BIT0_HIGH, default="300ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT0_LOW, default="950ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT1_HIGH, default="950ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_BIT1_LOW, default="300ns"): cv.positive_time_period_nanoseconds,
            cv.Optional(CONF_RESET_TIME, default="500us"): _validate_reset,
            # At rest a frame this often: keeps the boards' clocks in step and repeats the current state.
            cv.Optional(CONF_REFRESH_INTERVAL, default="200ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=20), max=cv.TimePeriod(seconds=10)),
            ),
            # How far ahead commands start, so every copy and every board starts together. auto: from the frame
            # time, the most lights on one board and what is queued; a Home Assistant change to a light group or a
            # scene starts at the same moment everywhere (also across data lines).
            cv.Optional(CONF_START_DELAY, default="auto"): _validate_start_delay,
            # Copies of every command (a corrupted frame does not lose it).
            cv.Optional("effect_rate", default=20): cv.int_range(min=1, max=20),
            cv.Optional("line_rate", default={}): cv.Schema({
                cv.Optional("automatic_step_down", default=True): cv.boolean,
                cv.Optional("window", default="5s"): cv.All(cv.positive_time_period_milliseconds, cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=60))),
                cv.Optional("minimum_frames", default=32): cv.int_range(min=1, max=65535),
                cv.Optional("error_ratio", default=0.01): cv.percentage,
                cv.Optional("bad_windows", default=3): cv.int_range(min=1, max=255),
                cv.Optional("inventory_complete", default=False): cv.boolean,
                cv.Optional("bootloader_profile_mask", default=1): cv.int_range(min=1, max=15),
                cv.Optional("path_distortion_ns", default=65535): cv.int_range(min=0, max=65535),
                cv.Optional("initial_profile", default=0): cv.int_range(min=0, max=3),
            }),
            cv.Optional("descriptors", default=[]): cv.ensure_list(cv.Schema({cv.Required("board"): board_position, cv.Required("file"): cv.file_})),
            # A Dig2Analog-Plus-5ch pin of one board model: "every board on this pin: this descriptor file" (you state
            # it; it is never inferred). A board's own descriptors: entry wins over it.
            cv.Optional(CONF_EVERY_BOARD_DESCRIPTOR): cv.file_,
            cv.Optional(CONF_REPEATS, default=3): cv.int_range(min=1, max=8),
            # Board settings (sent to every board; runtime setters change them per board)
            cv.Optional(CONF_PWM_FREQUENCY, default="25kHz"): _validate_pwm_frequency,
            cv.Optional(CONF_DITHERING, default="on"): _enum_or_bool(DITHERING, {True: "on", False: "off"}),
            cv.Optional(CONF_DITHER_THRESHOLD, default="1.0%"): _validate_dither_threshold,
            cv.Optional(CONF_DIM_FLOOR, default="skip"): _enum_or_bool(DIM_FLOOR, {False: "off"}),
            # Shortest pulse / off-time, 50ns - 1us. Not given: each board keeps its own (its firmware's default:
            # Dig2Analog-Plus-5ch rev 0.4 100ns, the others 200ns; a setting it stored keeps).
            cv.Optional(CONF_MIN_PULSE): _validate_min_pulse,
            # Below the minimum pulse a board skips pulses; the patterns repeat at least this often. 1kHz reaches
            # about half the lowest level of 2kHz with full-size pulses (from 64kHz PWM up: f_pwm / 64).
            cv.Optional(CONF_PULSE_SKIP_RATE, default="2kHz"): _validate_pulse_skip_rate,
            # off, fixed (channels a third of a period apart), fixed_spread, adaptive (phases follow the colour mix
            # for the least LED_+ ripple current; uses power_limit's channel_current as weights when given),
            # adaptive_spread (alias spread); *_spread adds spread-spectrum PWM (every period +-6 %, conducted EMC)
            cv.Optional(CONF_STAGGER, default="adaptive_spread"): _validate_stagger,
            cv.Optional(CONF_SIGNAL_LOSS, default="hold"): _enum_or_bool(SIGNAL_LOSS, {False: "off"}),
            cv.Optional(CONF_SIGNAL_LOSS_TIMEOUT, default="3s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(seconds=60)),
            ),
            # After an overcurrent trip a board switches its outputs on again trip_retry_interval later, at most
            # trip_retries times (0: never; forever); after the last one they stay off until a new command or a power
            # cycle, and 60 s without a trip starts the count again. Stored on the boards. Not given: the boards keep theirs (default 10 retries, 1 s apart).
            cv.Optional(CONF_TRIP_RETRIES): _validate_trip_retries,
            cv.Optional(CONF_TRIP_RETRY_INTERVAL): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=255)),
            ),
            cv.Optional(CONF_FEEDBACK_CHANNEL, default="red"): cv.enum(FEEDBACK, lower=True),
            cv.Optional(CONF_POWER_LIMIT): _validate_power_limit,
            cv.Optional(CONF_CURRENT_CALIBRATION): CURRENT_CALIBRATION_SCHEMA,
            # Diff-Power Ultra Receiver boards report back (receiver_schema.py): their power, energy, status,
            # problems and diagnostics, each receiver a Home Assistant device of its own. Off unless configured.
            cv.Optional(CONF_TELEMETRY): TELEMETRY_SCHEMA,
            # Protocol mode the boards start in after power-up; stored on the boards (the only setting that is),
            # written only when it differs. Not given: the boards keep what they have (default auto).
            cv.Optional(CONF_BOOT_PROTOCOL): cv.enum(BOOT_PROTOCOL, lower=True),
            # Supply of the boards / LEDs: requested while a light is on; the boards' state is sent again
            # power_on_delay after it switched on (fades that ran in the dark are replayed).
            cv.Optional(CONF_POWER_SUPPLY): cv.use_id(power_supply.PowerSupply),
            # or: the QuinLED Diff-Power port this line is on (diffpower): woken while a light here is on, and in
            # standby (4.5 V) after its turn-off period; everything is sent again once it is powered
            cv.Optional(CONF_PORT): cv.use_id(DiffPowerPort),
            cv.Optional(CONF_POWER_ON_DELAY, default="100ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(seconds=30)),
            ),
            cv.SplitDefault(
                CONF_RMT_SYMBOLS,
                esp32=192,
                esp32_c3=96,
                esp32_c5=96,
                esp32_c6=96,
                esp32_h2=96,
                esp32_p4=192,
                esp32_s2=192,
                esp32_s3=192,
            ): cv.int_range(min=2),
            cv.Optional(CONF_USE_DMA): cv.All(
                esp32.only_on_variant(supported=[esp32.VARIANT_ESP32P4, esp32.VARIANT_ESP32S3]),
                cv.boolean,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_transmitter,
    _validate_digital_line,
)


def _max_group_commands(lights):
    """One SET per light and, when following restore state, one stored power-on level."""
    counts = {}
    for light in lights:
        commands = 1 + int(light.get("power_on_scene", False))
        board = light["board"]
        counts[board] = counts.get(board, 0) + commands
    return max(counts.values(), default=1)


def _lights_of(full_config, hub_id):
    return [
        light
        for light in full_config.get("light", []) or []
        if light.get("platform") in ANALOG_LIGHT_PLATFORMS and light.get(CONF_ULTRA_LINE_ID) == hub_id
    ]


# RMT memory for transmitting per variant (ESP-IDF soc_caps): TX channels x symbols per memory block. A channel
# without DMA takes rmt_symbols / block size blocks, so the default 192 leaves room for 2 lines on an ESP32, 1 on an
# ESP32-S3 / C3 / C6.
RMT_TX_MEMORY = {
    esp32.VARIANT_ESP32: (8, 64),
    esp32.VARIANT_ESP32S2: (4, 64),
    esp32.VARIANT_ESP32S3: (4, 48),
    esp32.VARIANT_ESP32C3: (2, 48),
    esp32.VARIANT_ESP32C5: (2, 48),
    esp32.VARIANT_ESP32C6: (2, 48),
    esp32.VARIANT_ESP32H2: (2, 48),
    esp32.VARIANT_ESP32P4: (4, 48),
}


def _validate_rmt_memory(full):
    """The data lines (and other RMT transmitters) must fit the RMT memory, or the later ones fail at boot."""
    variant = esp32.get_esp32_variant()
    if variant not in RMT_TX_MEMORY:
        return
    channels, block = RMT_TX_MEMORY[variant]
    hubs = full.get("ultra_line", []) or []
    hubs = hubs if isinstance(hubs, list) else [hubs]
    users = [(f"ultra_line '{h[CONF_ID]}'", h) for h in hubs if CONF_PARLIO_ID not in h]   # (lanes: PARLIO)
    users += [
        (f"light '{c.get('name', c[CONF_ID])}'", c)
        for c in full.get("light", []) or []
        if c.get("platform") == "esp32_rmt_led_strip"
    ]
    users += [(f"remote_transmitter '{c[CONF_ID]}'", c) for c in full.get("remote_transmitter", []) or []]
    dma = [name for name, c in users if c.get(CONF_USE_DMA)]
    if len(dma) > 1:
        # (ESP-IDF rmt_tx.c: only the group's last transmit channel can use DMA)
        raise cv.Invalid(
            f"only one RMT transmitter of the {variant} can use DMA (its last channel); {', '.join(dma)} have "
            "use_dma: true. Give the others rmt_symbols instead"
        )
    need = 0
    for _, c in users:
        symbols = c.get(CONF_RMT_SYMBOLS, block)
        need += 1 if c.get(CONF_USE_DMA) else -(-symbols // block)
    if need > channels:
        names = ", ".join(name for name, _ in users)
        raise cv.Invalid(
            f"the RMT of the {variant} has {channels} x {block} symbols for transmitting; {names} need "
            f"{need} x {block}. Give the data lines a smaller rmt_symbols (e.g. {block} or {2 * block}; each line "
            f"takes rmt_symbols / {block} blocks)"
            + (" or use_dma: true" if variant in (esp32.VARIANT_ESP32S3, esp32.VARIANT_ESP32P4) else "")
        )


def _validate_line_speed(full, conf):
    """Every line: its port type's board limit. A Dig2Analog-Plus-5ch pin whose board count needs a slower speed than
    800 kHz needs a complete all-Ultra inventory for it (the sender's admission; never inferred from silence)."""
    port_type, marks = line_port_type(full, conf)
    if marks and port_type != "diff_power":
        raise cv.Invalid(
            f"ultra_line '{conf[CONF_ID]}': port_type: {port_type}, but the line is a Diff-Power port "
            f"({', '.join(marks)}): Diff-Power Ultra Receivers chain on an RS-485 port, at most 8 of them. Give "
            "port_type: diff_power, or leave it out",
            [CONF_PORT_TYPE],
        )
    boards = conf[CONF_NUM_BOARDS]
    if boards > PORT_MAX_BOARDS[port_type]:
        name = "a Diff-Power port" if port_type == "diff_power" else "a Dig2Analog-Plus-5ch data pin"
        raise cv.Invalid(
            f"ultra_line '{conf[CONF_ID]}' has num_boards: {boards}: {name} chains at most "
            f"{PORT_MAX_BOARDS[port_type]} boards (they forward without redriving)"
        )
    if port_type != "d2a_pin":
        if CONF_EVERY_BOARD_DESCRIPTOR in conf:
            raise cv.Invalid(
                f"ultra_line '{conf[CONF_ID]}': {CONF_EVERY_BOARD_DESCRIPTOR} is for a Dig2Analog-Plus-5ch data pin "
                "of one board model; a Diff-Power port's receivers report their own (or give descriptors: per board)",
                [CONF_EVERY_BOARD_DESCRIPTOR],
            )
        return
    profile = next(p for top, p in D2A_PIN_SPEEDS if boards <= top)
    if profile == 0:
        return
    policy = conf["line_rate"]
    given = range(boards) if CONF_EVERY_BOARD_DESCRIPTOR in conf else {board_slot(d["board"]) for d in conf.get("descriptors", [])}
    missing = sorted(set(range(boards)) - set(given))
    need = []
    if not policy["inventory_complete"]:
        need.append("line_rate: inventory_complete: true (only QuinLED Ultra boards on the pin, no pixels after them)")
    if not policy["bootloader_profile_mask"] & (1 << profile):
        need.append(f"line_rate: bootloader_profile_mask with profile {profile} ({1 << profile})")
    if missing:
        shown = positions_text(missing[:8]) + (", ..." if len(missing) > 8 else "")
        need.append(f"a descriptors: file for board{'s' if len(missing) > 1 else ''} {shown} (or "
                    f"{CONF_EVERY_BOARD_DESCRIPTOR}: one file for every board of the pin)")
    if need:
        raise cv.Invalid(
            f"ultra_line '{conf[CONF_ID]}': a Dig2Analog-Plus-5ch pin of {boards} boards runs at "
            f"{LINE_RATES[profile]} (1..20 boards 800 kHz, 21..34 600 kHz, 35..50 200 kHz), and leaving 800 kHz needs "
            f"a complete all-Ultra inventory: {'; '.join(need)}"
        )


def _validate_ports(full):
    """A line on a Diff-Power port: not also on a power_supply (the port powers it); one line per port."""
    hubs = full.get("ultra_line", []) or []
    seen = {}
    for conf in hubs if isinstance(hubs, list) else [hubs]:
        if CONF_PORT not in conf:
            continue
        if CONF_POWER_SUPPLY in conf:
            raise cv.Invalid(
                f"ultra_line '{conf[CONF_ID]}': port and power_supply both power the line; give one of them"
            )
        port = str(conf[CONF_PORT])
        if port in seen:
            raise cv.Invalid(
                f"ultra_line '{seen[port]}' and '{conf[CONF_ID]}' are both on port '{port}': a Diff-Power port "
                "carries one data line"
            )
        seen[port] = str(conf[CONF_ID])


def parlio_bus_of(full, conf):
    """The ultra_parlio bus a lane hub is on (None: an RMT line)."""
    if CONF_PARLIO_ID not in conf:
        return None
    for bus in full.get("ultra_parlio", []) or []:
        if str(bus[CONF_ID]) == str(conf[CONF_PARLIO_ID]):
            return bus
    return None


def parlio_hub_timing(full, conf):
    """A lane hub's timing on its bus' sample grid (bit0_high, bit0_low, bit1_high, bit1_low, reset; ns), so its
    frames' TIME is the stream's; None for an RMT line."""
    bus = parlio_bus_of(full, conf)
    if bus is None:
        return None
    hubs = full.get("ultra_line", []) or []
    keys = (CONF_BIT0_HIGH, CONF_BIT0_LOW, CONF_BIT1_HIGH, CONF_BIT1_LOW, CONF_RESET_TIME)
    lanes = {
        h[CONF_LANE]: tuple(int(h[k].total_nanoseconds) for k in keys)
        for h in (hubs if isinstance(hubs, list) else [hubs])
        if str(h.get(CONF_PARLIO_ID, "")) == str(bus[CONF_ID])
    }
    buffer = int(bus["buffer_time"].total_microseconds) * 1000 if "buffer_time" in bus else None
    lay = parlio_layout(bus["sample_rate"], len(bus["pins"]), lanes, buffer)
    return lay["lanes"].get(conf[CONF_LANE])


def _validate_lanes(full):
    """Each lane of a bus has one hub, within the bus' pins."""
    hubs = full.get("ultra_line", []) or []
    taken = {}
    for h in hubs if isinstance(hubs, list) else [hubs]:
        bus = parlio_bus_of(full, h)
        if bus is None:
            continue   # an RMT line (a missing bus: use_id says so)
        lane = h[CONF_LANE]
        if lane >= len(bus["pins"]):
            raise cv.Invalid(
                f"ultra_line '{h[CONF_ID]}': lane {lane}, but '{bus[CONF_ID]}' has {len(bus['pins'])} pins "
                f"(lanes 0-{len(bus['pins']) - 1})"
            )
        key = (str(bus[CONF_ID]), lane)
        if key in taken:
            raise cv.Invalid(
                f"ultra_line '{taken[key]}' and '{h[CONF_ID]}' are both on lane {lane} of '{bus[CONF_ID]}'"
            )
        taken[key] = str(h[CONF_ID])


def _raise_chain_first_reset(full, configs):
    """A normal line (an RMT line or a PARLIO lane) whose first board is an ur_digital_m receiver gets at least
    CHAIN_FIRST_RESET_NS of reset (chain_reset.py): the validated reset_time is raised, so the codegen and a PARLIO bus' layout see it."""
    for conf in configs if isinstance(configs, list) else [configs]:
        now = int(conf[CONF_RESET_TIME].total_nanoseconds)
        floor = reset_with_floor_ns(now, conf, full.get("light"))
        if floor != now:
            _LOGGER.info(
                "%s: reset_time raised from %d us to %d us: an ur_digital_m receiver is the first board of this line "
                "(chain mode: it classifies a frame while the next one is coming; behind another board the gap is 320 us longer)",
                conf[CONF_ID], now // 1000, CHAIN_FIRST_RESET_NS // 1000,
            )
            conf[CONF_RESET_TIME] = cv.positive_time_period_nanoseconds(f"{floor // 1000}us")


def _final_validate(configs):
    full = fv.full_config.get()
    hubs = full.get("ultra_line", []) or []
    first = configs[0] if isinstance(configs, list) else configs
    if hubs and first[CONF_ID] == hubs[0][CONF_ID]:
        warn_ram(full)   # (once: every line counts)
    _raise_chain_first_reset(full, configs)
    _validate_rmt_memory(full)
    _validate_lanes(full)
    validate_telemetry_lines(full)
    _validate_ports(full)
    _validate_commission_actions(full)
    for conf in configs if isinstance(configs, list) else [configs]:
        validate_telemetry(full, conf, conf[CONF_NUM_BOARDS])
        validate_board_platforms(full, conf)
        validate_channel_ceilings(full, conf)
        _validate_line_speed(full, conf)
        # signal_loss off / fade_off: a board switches off after signal_loss_timeout (sent in 100 ms) without a good
        # frame; at rest a frame comes every refresh_interval, so one lost frame must not reach the timeout
        refresh = int(conf[CONF_REFRESH_INTERVAL].total_milliseconds)
        timeout = round(conf[CONF_SIGNAL_LOSS_TIMEOUT].total_milliseconds / 100) * 100
        if conf[CONF_SIGNAL_LOSS] != "hold" and 2 * refresh > timeout:
            raise cv.Invalid(
                f"refresh_interval ({refresh} ms) must be at most half of signal_loss_timeout ({timeout} ms) with "
                f"signal_loss: {conf[CONF_SIGNAL_LOSS]}, or a single lost frame takes the lights to their signal loss "
                "state"
            )
        used = {}
        count = {}
        if CONF_DIGITAL in conf and _lights_of(full, conf[CONF_ID]):
            raise cv.Invalid(
                f"'{conf[CONF_ID]}' is a digital port: it has pixel lights (ur_digital_m, ur_digital_m_stream), not "
                "analog lights (dig2analog_plus, ur_analog_m)"
            )
        for light in _lights_of(full, conf[CONF_ID]):
            board = light["board"]
            if board > conf[CONF_NUM_BOARDS]:
                raise cv.Invalid(
                    f"light '{light.get('name', light[CONF_ID])}': "
                    + out_of_range(board, conf[CONF_ID], conf[CONF_NUM_BOARDS])
                )
            count[board] = count.get(board, 0) + 1
            for ch in (light[k] for k in ROLE_KEYS if k in light):
                key = (board, ch)
                if key in used:
                    raise cv.Invalid(
                        f"board {board} CH{ch + 1} is used by both '{used[key]}' and "
                        f"'{light.get('name', light[CONF_ID])}'"
                    )
                used[key] = light.get("name", str(light[CONF_ID]))
        for b in conf.get(CONF_CURRENT_CALIBRATION, {}).get(CONF_BOARDS, []):
            if b > conf[CONF_NUM_BOARDS]:
                raise cv.Invalid("current_calibration: " + out_of_range(b, conf[CONF_ID], conf[CONF_NUM_BOARDS]))
        if CONF_POWER_SUPPLY in conf:
            for psu in full.get("power_supply", []) or []:
                if psu[CONF_ID] == conf[CONF_POWER_SUPPLY] and psu[CONF_ENABLE_TIME].total_milliseconds > 50:
                    _LOGGER.warning(
                        "ultra_line: power_supply '%s' has enable_time %d ms, which blocks the main loop "
                        "on every switch-on; set enable_time: 0ms and use power_on_delay instead",
                        conf[CONF_POWER_SUPPLY],
                        psu[CONF_ENABLE_TIME].total_milliseconds,
                    )
    # ultra_* effects only run on analog lights (dig2analog_plus, ur_analog_m)
    for light in full.get("light", []) or []:
        if light.get("platform") in ANALOG_LIGHT_PLATFORMS + ("ur_digital_m",):
            continue
        for eff in light.get("effects", []) or []:
            key = next(iter(eff))
            if key.startswith("ultra_"):
                raise cv.Invalid(
                    f"light '{light.get('name', light[CONF_ID])}': effect {key} only runs on analog lights (dig2analog_plus, ur_analog_m)"
                )
    return configs


FINAL_VALIDATE_SCHEMA = _final_validate


# ------------------------------------------------------------------------------------------- the commissioning
# Commission / Finalize a line with telemetry (wiki: Commissioning), or every one (no id); one line at a time
def _commission_lines(full):
    return {str(h[CONF_ID].id) for h in full.get("ultra_line", []) or []
            if h.get(CONF_TELEMETRY) and h[CONF_TELEMETRY].get("commissioning", True)}


def _validate_commission_actions(full):
    """Each commission / finalize action's line: one with telemetry and commissioning (else the action does nothing)."""
    lines = _commission_lines(full)
    for name, hub in CORE.data.get("ultra_line_commission_actions", []):
        if hub is None and not lines:
            raise cv.Invalid(f"ultra_line.{name}: no line has telemetry with commissioning")
        if hub is not None and hub not in lines:
            raise cv.Invalid(f"ultra_line.{name}: line '{hub}' has no telemetry with commissioning")


def _commission_action(name):
    def validate(config):
        hub = config.get(CONF_ID)
        CORE.data.setdefault("ultra_line_commission_actions", []).append(
            (name, str(hub.id) if hub is not None else None)
        )
        return config

    return cv.All(automation.maybe_simple_id({cv.Optional(CONF_ID): cv.use_id(UltraHub)}), validate)


async def _commission_to_code(config, action_id, template_arg, finalize):
    var = cg.new_Pvariable(action_id, template_arg)
    if CONF_ID in config:
        cg.add(var.set_hub(await cg.get_variable(config[CONF_ID])))
    cg.add(var.set_finalize(finalize))
    return var


@automation.register_action(
    "ultra_line.commission", CommissionAction, _commission_action("commission"), synchronous=True
)
async def commission_to_code(config, action_id, template_arg, args):
    return await _commission_to_code(config, action_id, template_arg, False)


@automation.register_action(
    "ultra_line.finalize", CommissionAction, _commission_action("finalize"), synchronous=True
)
async def finalize_to_code(config, action_id, template_arg, args):
    return await _commission_to_code(config, action_id, template_arg, True)


def _keep_on_time_ms(full_config, psu_id) -> int:
    for conf in full_config.get("power_supply", []) or []:
        if conf[CONF_ID] == psu_id:
            return int(conf[CONF_KEEP_ON_TIME].total_milliseconds)
    return 0


async def to_code(config: ConfigType) -> None:
    include_builtin_idf_component("esp_driver_rmt")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # the receivers' settings from Home Assistant (wiki: Controller-Settings): the line's id (their record on the
    # ESP32, the action's port) and the action set_receiver_setting (with api:, one for every line)
    cg.add(var.set_line_id(str(config[CONF_ID].id)))
    if "api" in CORE.config:
        cg.add_define("USE_API_USER_DEFINED_ACTIONS")
        cg.add_define("USE_API_CUSTOM_SERVICES")

    timing = tuple(
        int(config[k].total_nanoseconds)
        for k in (CONF_BIT0_HIGH, CONF_BIT0_LOW, CONF_BIT1_HIGH, CONF_BIT1_LOW, CONF_RESET_TIME)
    )
    if CONF_PARLIO_ID in config:
        # a lane: the bus' pin, and the timing on its sample grid (the frames' TIME is the stream's)
        bus = await cg.get_variable(config[CONF_PARLIO_ID])
        cg.add(var.set_parlio_lane(bus.lane(config[CONF_LANE])))
        cg.add(var.set_parlio_bus(bus))
        pin = parlio_bus_of(CORE.config, config)["pins"][config[CONF_LANE]]
        timing = parlio_hub_timing(CORE.config, config)
    else:
        pin = config[CONF_PIN]
    cg.add(var.set_pin(pin[CONF_NUMBER]))
    if pin[CONF_INVERTED]:
        cg.add(var.set_inverted(True))
    cg.add(var.set_num_boards(config[CONF_NUM_BOARDS]))
    def descriptor_bytes(file):
        from pathlib import Path
        blob = Path(CORE.relative_config_path(file)).read_bytes()
        if len(blob) < 11 or len(blob) > 6144:
            raise cv.Invalid("descriptor file must contain 11..6144 bytes exported by the board firmware")
        return list(blob)

    if CONF_EVERY_BOARD_DESCRIPTOR in config:
        cg.add(var.set_every_board_descriptor(descriptor_bytes(config[CONF_EVERY_BOARD_DESCRIPTOR])))
    for inventory in config["descriptors"]:
        cg.add(var.set_board_descriptor(board_slot(inventory["board"]), descriptor_bytes(inventory["file"])))

    # lights per board: the hub clears the other light indices on the boards (left by an earlier configuration)
    lights = _lights_of(CORE.config, config[CONF_ID])
    cg.add(var.set_max_commands(_max_group_commands(lights)))
    cg.add(var.set_light_counts([
        sum(board_slot(light["board"]) == b for light in lights) for b in range(config[CONF_NUM_BOARDS])
    ]))
    cg.add(var.set_rmt_symbols(config[CONF_RMT_SYMBOLS]))
    if CONF_USE_DMA in config:
        cg.add(var.set_use_dma(config[CONF_USE_DMA]))
    cg.add(var.set_timing(*timing))
    cg.add(var.set_effect_rate(config["effect_rate"]))
    policy = config["line_rate"]
    cg.add(var.set_line_rate_policy(policy["automatic_step_down"], policy["window"].total_milliseconds,
                                   policy["minimum_frames"], round(policy["error_ratio"] * 1000), policy["bad_windows"],
                                   policy["inventory_complete"], policy["bootloader_profile_mask"],
                                   policy["path_distortion_ns"], policy["initial_profile"]))
    # the port's type: its board limit, the line speed its board count starts at (the update entity reads it here)
    cg.add(var.set_port_type(PORT_TYPES[line_port_type(CORE.config, config)[0]]))
    cg.add(var.set_refresh_interval(config[CONF_REFRESH_INTERVAL].total_milliseconds))
    if CONF_DIGITAL in config:
        cg.add_define("USE_DIG2ANALOG_UR_DIGITAL_M")
        cg.add(var.set_digital(int(config[CONF_DIGITAL][CONF_RESET_TIME].total_nanoseconds // 1000)))
    if config[CONF_START_DELAY] != "auto":
        cg.add(var.set_start_delay(config[CONF_START_DELAY].total_milliseconds))
    cg.add(var.set_repeats(config[CONF_REPEATS]))

    # board settings: all boards (-1), sent at boot
    cg.add(var.set_pwm_frequency(-1, config[CONF_PWM_FREQUENCY]))
    cg.add(var.set_board_config(-1, CFG_DITHER, DITHERING[config[CONF_DITHERING]]))
    cg.add(var.set_board_config(-1, CFG_DITHER_THRESH, round(config[CONF_DITHER_THRESHOLD] * 1000)))
    cg.add(var.set_board_config(-1, CFG_DIM_FLOOR, DIM_FLOOR[config[CONF_DIM_FLOOR]]))
    if CONF_MIN_PULSE in config:
        cg.add(var.set_board_config(-1, CFG_MIN_PULSE, round(config[CONF_MIN_PULSE].total_nanoseconds / 10)))
    # always sent (a restart puts back 2 kHz); a board whose descriptor lacks SKIP_RATE drops it (ultra_descriptor_config)
    cg.add(var.set_board_config(-1, CFG_SKIP_RATE, config[CONF_PULSE_SKIP_RATE]))
    cg.add(var.set_board_config(-1, CFG_STAGGER, STAGGER[config[CONF_STAGGER]]))
    cg.add(var.set_board_config(-1, CFG_LOSS, SIGNAL_LOSS[config[CONF_SIGNAL_LOSS]]))
    cg.add(var.set_board_config(-1, CFG_LOSS_TIMEOUT, round(config[CONF_SIGNAL_LOSS_TIMEOUT].total_milliseconds / 100)))
    cg.add(var.set_board_config(-1, CFG_FEEDBACK, FEEDBACK[config[CONF_FEEDBACK_CHANNEL]]))
    plim = config.get(CONF_POWER_LIMIT, {})
    for idx, key in enumerate(CHANNEL_KEYS):
        amps = plim.get(CONF_CHANNEL_CURRENT, {}).get(key, 0.0)
        cg.add(var.set_channel_config(-1, idx, CFG_CH_CURRENT, round(amps * 1000)))
        limit = plim.get(CONF_MAX_CHANNEL_CURRENT, {}).get(key, 0.0)
        cg.add(var.set_channel_config(-1, idx, CFG_CH_MAX, round(limit * 1000)))
    cg.add(var.set_board_config(-1, CFG_BOARD_MAX, round(plim.get(CONF_MAX_BOARD_CURRENT, 0.0) * 1000)))
    if CONF_BOOT_PROTOCOL in config:
        cg.add(var.set_board_config(-1, CFG_BOOT_PROTOCOL, BOOT_PROTOCOL[config[CONF_BOOT_PROTOCOL]]))
    if CONF_TRIP_RETRIES in config or CONF_TRIP_RETRY_INTERVAL in config:
        count = config.get(CONF_TRIP_RETRIES, TRIP_RETRIES_DEFAULT)
        interval = int(config[CONF_TRIP_RETRY_INTERVAL].total_seconds) if CONF_TRIP_RETRY_INTERVAL in config else 1
        cg.add(var.set_board_config(-1, CFG_RETRY, count | interval << 8))

    if CONF_PORT in config:
        cg.add(var.set_port(await cg.get_variable(config[CONF_PORT])))
    if CONF_POWER_SUPPLY in config:
        psu = await cg.get_variable(config[CONF_POWER_SUPPLY])
        cg.add(var.set_power_supply(psu, _keep_on_time_ms(CORE.config, config[CONF_POWER_SUPPLY])))
        cg.add(var.set_power_on_delay(config[CONF_POWER_ON_DELAY].total_milliseconds))
    if CONF_TELEMETRY in config:
        await telemetry_to_code(var, config)
    if CONF_CURRENT_CALIBRATION in config:
        cal = config[CONF_CURRENT_CALIBRATION]
        cg.add(
            var.set_calibration_schedule(
                cal[CONF_ON_BOOT],
                int(cal[CONF_INTERVAL].total_milliseconds) if CONF_INTERVAL in cal else 0,
                cal[CONF_ONLY_WHEN_OFF],
                [board_slot(b) for b in cal.get(CONF_BOARDS, [])],
            )
        )


# ------------------------------------------------------------------------------- switch WS-forced boards to Ultra
SWITCH_BOOT_PROTOCOL = {"auto": 0, "ultra": 3}


def _validate_switch_boot_protocol(value):
    """auto or ultra: what the boards store as their boot protocol after the switch."""
    key = cv.string_strict(value).lower()
    if key not in SWITCH_BOOT_PROTOCOL:
        raise cv.Invalid(f"boot_protocol must be auto or ultra, not '{value}'")
    return SWITCH_BOOT_PROTOCOL[key]


@automation.register_action(
    "ultra_line.switch_boards_to_ultra",
    SwitchBoardsToUltraAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(UltraHub),
            cv.Optional(CONF_BOOT_PROTOCOL, default="auto"): cv.templatable(_validate_switch_boot_protocol),
        }
    ),
    synchronous=True,
)
async def switch_boards_to_ultra_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_boot_protocol(await cg.templatable(config[CONF_BOOT_PROTOCOL], args, cg.uint8)))
    return var


# ------------------------------------------------------------------------------- current calibration on demand
# The pin goes dark, its boards are tested one after the other in chain order (~1.4 s each), then the pin fades back.
# Other pins keep running.
@automation.register_action(
    "ultra_line.calibrate_current",
    CalibrateCurrentAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(UltraHub),
            # which boards to test (1 = the first board), default all on this pin (the whole pin goes dark either way)
            cv.Optional(CONF_BOARDS): cv.ensure_list(board_position),
            # all = the standard test (each channel at 25 / 50 / 75 / 100 %); or one level, e.g. 25%
            cv.Optional(CONF_LEVEL, default="all"): _validate_cal_level,
        }
    ),
    synchronous=True,
)
async def calibrate_current_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_boards([board_slot(b) for b in config.get(CONF_BOARDS, [])]))
    cg.add(var.set_level(config[CONF_LEVEL]))
    return var


# A digital port's receiver stopped answering (the hub tried 3 polls, 60 s later 3 more, and keeps the strip free of
# markers since): poll it again.
@automation.register_action(
    "ultra_line.retry_receiver",
    RetryReceiverAction,
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(UltraHub)}),
    synchronous=True,
)
async def retry_receiver_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "ultra_line.cancel_calibration",
    CancelCalibrationAction,
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(UltraHub)}),
    synchronous=True,
)
async def cancel_calibration_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


# ------------------------------------------------------------------------------- firmware version as blinks
# Each board (or one) blinks its firmware version once on its feedback channel (IDENTIFY 5).
@automation.register_action(
    "ultra_line.show_version",
    ShowVersionAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(UltraHub),
            # one board of this line (1 = the first board); left out: every board
            cv.Optional("board"): cv.templatable(board_position),
        }
    ),
    synchronous=True,
)
async def show_version_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    if "board" in config:
        cg.add(var.set_board(await cg.templatable(config["board"], args, cg.int_)))
    return var
