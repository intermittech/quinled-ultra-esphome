"""Schema tests: run `esphome config` on small configs and check pass/fail and messages.

Usage: .venv/Scripts/python tests/validate_configs.py [name filter ...]
       (with filters, only the cases whose name contains one of them run)
"""

from pathlib import Path
import subprocess
import sys
import textwrap

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = HERE / "build" / "cfg"
ESPHOME = [sys.executable, "-m", "esphome"]

BASE = """\
esphome:
  name: t
esp32:
  board: {board}
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [dig2analog_ultra, dig2analog_ultra_strip]
"""

PSU = """\
power_supply:
  - id: psu
    pin: GPIO13
    enable_time: {enable}
"""

LEDC = """\
output:
  - platform: ledc
    id: o1
    pin: GPIO25
"""

ULTRA = """\
dig2analog_ultra:
  id: u
  pin: GPIO16
  num_boards: 2
"""

ULTRA_RGBWW = """\
light:
  - platform: dig2analog_ultra
    name: rgbww
    board: 0
    red: 1
    green: 2
    blue: 3
    warm_white: 4
    cold_white: 5
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
    effects:
      - ultra_pulse:
      - ultra_color_loop:
      - ultra_strobe:
      - ultra_sunrise:
          duration: 10min
"""

ULTRA_CCT = """\
light:
  - platform: dig2analog_ultra
    name: cct
    board: 0
    warm_white: 1
    cold_white: 2
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
"""

ULTRA_MONO = """\
  - platform: dig2analog_ultra
    name: {name}
    board: {board}
    channel: {channel}
"""

MONO_CANDLE = """\
  - platform: dig2analog_ultra
    name: niche
    board: 0
    channel: 1
    effects:
      - ultra_candle:
"""

STRIP_PIXELS = """\
light:
  - platform: dig2analog_ultra
    id: d1
    name: d1
    board: 0
    channel: 1
  - platform: dig2analog_ultra
    id: d2
    name: d2
    board: 0
    channel: 2
"""

STRIP = """\
  - platform: dig2analog_ultra_strip
    id: s
    name: strip
    lights: [{lights}]
    effects:
      - addressable_scan:
      - strobe:
"""

PARTITION = """\
  - platform: partition
    name: p
{gamma}    segments:
      - id: s
        from: 0
        to: 1
"""


def lines(n, symbols=None):
    out = "dig2analog_ultra:\n"
    for i in range(n):
        out += f"  - id: u{i}\n    pin: GPIO{16 + i}\n    num_boards: 1\n"
        if symbols:
            out += f"    rmt_symbols: {symbols}\n"
    return out


# board firmware update entity (update.py)
UPD_NET = "wifi:\n  ssid: x\n  password: xxxxxxxx\nhttp_request:\n"
UPD = "update:\n  - platform: dig2analog_ultra\n    id: fw\n    name: fw\n    dig2analog_ultra_id: u\n"

# (name, yaml body, expect_ok, substring expected in output or None, board)
CASES = [
    ("ultra minimal hub", ULTRA, True, None),
    # officially supported: up to 50 boards per data pin (research/chain-50-boards.md); the schema allows 64
    (
        "ultra 50 boards, lights on boards 0 and 49",
        ULTRA.replace("num_boards: 2", "num_boards: 50")
        + "light:\n"
        + ULTRA_MONO.format(name="first", board=0, channel=1)
        + ULTRA_MONO.format(name="last", board=49, channel=5),
        True,
        None,
    ),
    (
        "ultra 50 boards, a light on board 50",
        ULTRA.replace("num_boards: 2", "num_boards: 50") + "light:\n" + ULTRA_MONO.format(name="x", board=50, channel=1),
        False,
        "boards are 0-based",
    ),
    ("ultra num_boards 65", ULTRA.replace("num_boards: 2", "num_boards: 65"), False, "64"),
    ("ultra rgbww light with board effects", ULTRA + ULTRA_RGBWW, True, None),
    (
        "ultra monochromatic lights on board 1",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + ULTRA_MONO.format(name="b", board=1, channel="ch2"),
        True,
        None,
    ),
    (
        "ultra board out of range",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=2, channel=1),
        False,
        "board 2 out of range",
    ),
    (
        "ultra channel used twice",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + ULTRA_MONO.format(name="b", board=0, channel=1),
        False,
        "is used by both",
    ),
    (
        "ultra incomplete rgb",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: x\n    red: 1\n    green: 2\n",
        False,
        "give the channels of one light type",
    ),
    (
        "ultra constant_brightness on rgb",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: x\n    red: 1\n    green: 2\n    blue: 3\n"
        "    constant_brightness: true\n",
        False,
        "constant_brightness needs cold_white and warm_white",
    ),
    (
        "ultra color loop on a monochromatic light",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    effects:\n      - ultra_color_loop:\n",
        False,
        "needs an RGB light",
    ),
    (
        "ultra effect on a non-Ultra light",
        LEDC + ULTRA
        + "light:\n  - platform: monochromatic\n    name: m\n    output: o1\n    effects:\n      - ultra_pulse:\n"
        + ULTRA_MONO.format(name="a", board=0, channel=1),
        False,
        "only runs on dig2analog_ultra lights",
    ),
    ("ultra pwm_frequency 150 kHz", ULTRA + "  pwm_frequency: 150kHz\n", False, "pwm_frequency must be"),
    ("ultra pwm_frequency 33 kHz", ULTRA + "  pwm_frequency: 33kHz\n", True, None),
    # start_delay: auto (default) or a fixed time up to 1 s
    ("ultra start_delay auto", ULTRA + "  start_delay: auto\n", True, None),
    ("ultra start_delay fixed", ULTRA + "  start_delay: 50ms\n", True, None),
    ("ultra start_delay too long", ULTRA + "  start_delay: 2s\n", False, "start_delay must be auto or at most 1s"),
    ("ultra start_delay bogus", ULTRA + "  start_delay: soon\n", False, "start_delay"),
    # the line's diagnostics: optional sensors
    (
        "ultra diagnostics sensors",
        ULTRA + "sensor:\n  - platform: dig2analog_ultra\n    update_interval: 10s\n    queue_peak:\n      name: q\n"
        "    memory_peak:\n      name: m\n    frame_build_time:\n      name: b\n    rmt_underruns:\n      name: u\n"
        "    frame_rate:\n      name: r\n",
        True,
        None,
    ),
    (
        "ultra diagnostics unknown sensor",
        ULTRA + "sensor:\n  - platform: dig2analog_ultra\n    heap_free:\n      name: h\n",
        False,
        "heap_free",
    ),
    (
        "ultra settings",
        ULTRA + "  dithering: off_at_low_levels\n  dither_threshold: 2%\n  dim_floor: clamp\n  min_pulse: 150ns\n"
        "  signal_loss: fade_off\n  signal_loss_timeout: 5s\n  feedback_channel: all\n  stagger: false\n",
        True,
        None,
    ),
    ("ultra min_pulse 20 ns", ULTRA + "  min_pulse: 20ns\n", False, "min_pulse must be"),
    # YAML reads a bare on / off as a boolean: the modes that are called on / off take it
    ("ultra bare on / off (yaml booleans)", ULTRA + "  dithering: on\n  signal_loss: off\n  dim_floor: off\n", True, None),
    ("ultra bare on for signal_loss", ULTRA + "  signal_loss: on\n", False, "'on' is not an option here; use one of hold, off, fade_off"),
    ("ultra dithering bogus", ULTRA + "  dithering: maybe\n", False, "Unknown value 'maybe'"),
    ("ultra stagger adaptive", ULTRA + "  stagger: adaptive\n", True, None),
    ("ultra stagger adaptive_spread", ULTRA + "  stagger: adaptive_spread\n", True, None),
    ("ultra stagger spread", ULTRA + "  stagger: spread\n", True, None),
    ("ultra stagger fixed_spread", ULTRA + "  stagger: fixed_spread\n", True, None),
    ("ultra stagger true", ULTRA + "  stagger: true\n", True, None),
    ("ultra stagger bogus", ULTRA + "  stagger: sideways\n", False, "Unknown value"),
    (
        "ultra power_limit without limits",
        ULTRA + "  power_limit:\n    channel_current: 2A\n",
        False,
        "power_limit needs",
    ),
    (
        "ultra power_limit",
        ULTRA + "  power_limit:\n    channel_current: {default: 2A, ch4: 4A, ch5: 4A}\n    max_channel_current: 5A\n"
        "    max_board_current: 8A\n",
        True,
        None,
    ),
    (
        "ultra power_limit measured board current only",
        ULTRA + "  power_limit:\n    max_board_current: 6A\n",
        True,
        None,
    ),
    (
        "ultra max_channel_current without channel_current",
        ULTRA + "  power_limit:\n    max_channel_current: 3A\n",
        False,
        "max_channel_current needs channel_current",
    ),
    # currents go to the boards in mA (16 bit)
    (
        "ultra channel_current negative",
        ULTRA + "  power_limit:\n    channel_current: -1A\n    max_channel_current: 2A\n",
        False,
        "value must be at least",
    ),
    (
        "ultra channel_current above 65.535 A",
        ULTRA + "  power_limit:\n    channel_current: 100A\n    max_channel_current: 2A\n",
        False,
        "value must be at most",
    ),
    (
        "ultra max_channel_current above 65.535 A",
        ULTRA + "  power_limit:\n    channel_current: 2A\n    max_channel_current: 100A\n",
        False,
        "value must be at most",
    ),
    (
        "ultra power_limit currents at 0 and 65.535 A",
        ULTRA + "  power_limit:\n    channel_current: {default: 65.535A, ch5: 0A}\n    max_channel_current: 65.535A\n",
        True,
        None,
    ),
    # signal_loss off / fade_off: a frame at least every half signal_loss_timeout (as sent: x 100 ms)
    (
        "ultra signal_loss with a slow refresh",
        ULTRA + "  refresh_interval: 2s\n  signal_loss: fade_off\n  signal_loss_timeout: 500ms\n",
        False,
        "refresh_interval (2000 ms) must be at most half of signal_loss_timeout (500 ms)",
    ),
    (
        "ultra signal_loss refresh at half the timeout",
        ULTRA + "  refresh_interval: 250ms\n  signal_loss: \"off\"\n  signal_loss_timeout: 500ms\n",
        True,
        None,
    ),
    (
        "ultra signal_loss timeout as sent (549 ms = 500 ms)",
        ULTRA + "  refresh_interval: 260ms\n  signal_loss: \"off\"\n  signal_loss_timeout: 549ms\n",
        False,
        "must be at most half of signal_loss_timeout (500 ms)",
    ),
    (
        "ultra signal_loss hold with a slow refresh",
        ULTRA + "  refresh_interval: 2s\n  signal_loss: hold\n  signal_loss_timeout: 500ms\n",
        True,
        None,
    ),
    (
        "ultra calibrate_current lambda",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - lambda: 'id(u).calibrate_current(0);'\n",
        True,
        None,
    ),
    (
        "ultra calibrate_current action",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          boards: [1]\n          level: 50%\n"
        "      - dig2analog_ultra.cancel_calibration: u\n",
        True,
        None,
    ),
    (
        "ultra show_version action",
        ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
        "      - dig2analog_ultra.show_version: u\n"
        "      - dig2analog_ultra.show_version:\n          id: u\n          board: 1\n"
        "      - dig2analog_ultra.show_version:\n          id: u\n          board: !lambda 'return 0;'\n",
        True,
        None,
    ),
    (
        "ultra show_version board out of range",
        ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
        "      - dig2analog_ultra.show_version:\n          id: u\n          board: 64\n",
        False,
        "value must be at most 63",
    ),
    (
        "ultra calibrate_current sequential (removed: always one board at a time)",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          sequential: true\n",
        False,
        "[sequential] is an invalid option",
    ),
    (
        "ultra calibrate_current level 3 %",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - dig2analog_ultra.calibrate_current:\n          id: u\n          level: 3%\n",
        False,
        "level must be all",
    ),
    (
        "ultra current_calibration schedule",
        ULTRA + "  current_calibration:\n    on_boot: true\n    interval: 7d\n    only_when_off: true\n"
        "    boards: [0, 1]\n",
        True,
        None,
    ),
    (
        "ultra current_calibration schedule with a power supply",
        PSU.format(enable="0ms") + ULTRA + "  power_supply: psu\n  current_calibration:\n    interval: 24h\n",
        True,
        None,
    ),
    (
        "ultra current_calibration every minute",
        ULTRA + "  current_calibration:\n    interval: 1min\n",
        False,
        "value must be at least",
    ),
    (
        "ultra current_calibration board out of range",
        ULTRA + "  current_calibration:\n    on_boot: true\n    boards: [2]\n",
        False,
        "current_calibration: board 2 out of range",
    ),
    (
        "ultra supply with blocking enable_time",
        PSU.format(enable="1500ms") + ULTRA + "  power_supply: psu\n",
        True,
        "enable_time 1500 ms, which blocks the main loop",
    ),
    ("ultra boot_protocol ultra", ULTRA + "  boot_protocol: ultra\n", True, None),
    ("ultra boot_protocol WS2805", ULTRA + "  boot_protocol: WS2805\n", True, None),
    ("ultra boot_protocol bogus", ULTRA + "  boot_protocol: dmx\n", False, "Unknown value"),
    (
        "ultra switch_boards_to_ultra action",
        ULTRA + "button:\n  - platform: template\n    name: s\n    on_press:\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ultra\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: !lambda 'return 0;'\n",
        True,
        None,
    ),
    (
        "ultra switch_boards_to_ultra to WS2805",
        ULTRA + "button:\n  - platform: template\n    name: s\n    on_press:\n"
        "      - dig2analog_ultra.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ws2805\n",
        False,
        "boot_protocol must be auto or ultra",
    ),
    (
        "ultra power_on_scene with restore",
        ULTRA + ULTRA_RGBWW + "    restore_mode: RESTORE_DEFAULT_OFF\n    power_on_scene: true\n",
        True,
        None,
    ),
    (
        "ultra power_on_scene ALWAYS_ON initial_state",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    restore_mode: ALWAYS_ON\n    initial_state:\n      brightness: 40%\n    power_on_scene: true\n",
        True,
        None,
    ),
    (
        "ultra power_on_scene with ALWAYS_OFF",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    power_on_scene: true\n",
        False,
        "power_on_scene shows the state the light comes back in",
    ),
    # power_on_effect: one of the light's ultra_* effects, run by the board from its power-up (SCENE_EFFECT)
    ("ultra power_on_effect by name", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_effect: Candle\n", True, None),
    (
        "ultra power_on_effect colour temperature warmer than the WW LED",
        ULTRA + ULTRA_CCT + "    effects:\n      - ultra_candle:\n          intensity: 30%\n"
        "    restore_mode: RESTORE_DEFAULT_OFF\n"
        "    power_on_effect:\n      effect: candle\n      brightness: 60%\n      color_temperature: 2200 K\n",
        False,
        "color_temperature must be between the LEDs' 2700 K and 6500 K",
    ),
    (
        "ultra power_on_effect colour temperature on cct",
        ULTRA + ULTRA_CCT + "    effects:\n      - ultra_candle:\n"
        "    power_on_effect:\n      effect: candle\n      brightness: 60%\n      color_temperature: 2700 K\n",
        True,
        None,
    ),
    (
        "ultra power_on_effect colour on rgbww",
        ULTRA + ULTRA_RGBWW + "    power_on_effect:\n      effect: Color Loop\n      brightness: 50%\n"
        "      red: 100%\n      green: 40%\n",
        True,
        None,
    ),
    (
        "ultra power_on_effect sunrise on rgbww",
        ULTRA + ULTRA_RGBWW + "    power_on_effect:\n      effect: Sunrise\n      color_temperature: 3000 K\n",
        True,
        None,
    ),
    (
        "ultra power_on_effect unknown effect",
        ULTRA + "light:\n" + MONO_CANDLE + "    power_on_effect: Fire\n",
        False,
        "this light has no effect 'Fire'; use one of: Candle",
    ),
    (
        "ultra power_on_effect without effects",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    power_on_effect: Candle\n",
        False,
        "add one under effects:",
    ),
    (
        "ultra power_on_effect strobe",
        ULTRA + ULTRA_RGBWW + "    power_on_effect: Strobe\n",
        False,
        "doesn't store the steps of 'Strobe' (ultra_strobe) and would show the plain values",
    ),
    (
        "ultra power_on_effect ESPHome effect",
        ULTRA + "light:\n" + MONO_CANDLE + "      - flicker:\n    power_on_effect: Flicker\n",
        False,
        "'Flicker' is an ESPHome effect (flicker)",
    ),
    (
        "ultra power_on_effect with power_on_scene",
        ULTRA + "light:\n" + MONO_CANDLE
        + "    restore_mode: RESTORE_DEFAULT_OFF\n    power_on_scene: true\n    power_on_effect: Candle\n",
        False,
        "remove power_on_scene",
    ),
    (
        "ultra power_on_effect brightness 0 %",
        ULTRA + "light:\n" + MONO_CANDLE + "    power_on_effect:\n      effect: Candle\n      brightness: 0%\n",
        False,
        "brightness must be above 0 %",
    ),
    (
        "ultra power_on_effect colour temperature on a monochromatic light",
        ULTRA + "light:\n" + MONO_CANDLE + "    power_on_effect:\n      effect: Candle\n      color_temperature: 2700 K\n",
        False,
        "color_temperature needs cold_white and warm_white channels",
    ),
    (
        "ultra power_on_effect red on cct",
        ULTRA + ULTRA_CCT + "    effects:\n      - ultra_candle:\n"
        "    power_on_effect:\n      effect: Candle\n      red: 50%\n",
        False,
        "red needs red, green and blue channels",
    ),
    (
        "ultra power_on_effect colour temperature and cold_white",
        ULTRA + ULTRA_CCT + "    effects:\n      - ultra_candle:\n"
        "    power_on_effect:\n      effect: Candle\n      color_temperature: 3000 K\n      cold_white: 50%\n",
        False,
        "give color_temperature or cold_white / warm_white, not both",
    ),
    (
        "ultra power_on_effect RGB and white with color_interlock",
        ULTRA + ULTRA_RGBWW + "    color_interlock: true\n"
        "    power_on_effect:\n      effect: Pulse\n      red: 100%\n      color_temperature: 3000 K\n",
        False,
        "with color_interlock give either the RGB colour or the white values",
    ),
    ("ultra dim_to_warm true on cct", ULTRA + ULTRA_CCT + "    dim_to_warm: true\n", True, None),
    (
        "ultra dim_to_warm warm end on cct",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 2200 K\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm red_assist on rgbww",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      warm_color_temperature: 1800 K\n      red_assist: 25%\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm red_assist on cct",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 1800 K\n      red_assist: 25%\n",
        False,
        "red_assist needs an RGBCCT light",
    ),
    (
        "ultra dim_to_warm red_assist above the WW LED",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      warm_color_temperature: 3000 K\n      red_assist: 25%\n",
        False,
        "red_assist only works below the warm white LED",
    ),
    (
        "ultra dim_to_warm on a monochromatic light",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1) + "    dim_to_warm: true\n",
        False,
        "dim_to_warm needs cold_white and warm_white",
    ),
    (
        "ultra dim_to_warm points (Kelvin and mireds)",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 100%\n        color_temperature: 4000 K\n"
        "      - brightness: 20%\n        color_temperature: 3000K\n"
        "      - brightness: 1%\n        color_temperature: 370 mireds\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm points with red assist",
        ULTRA + ULTRA_RGBWW + "    dim_to_warm:\n      red_assist: 30%\n      points:\n"
        "      - brightness: 100%\n        color_temperature: 2700 K\n"
        "      - brightness: 20%\n        color_temperature: 2200 K\n"
        "      - brightness: 1%\n        color_temperature: 1800 K\n",
        True,
        None,
    ),
    (
        "ultra dim_to_warm points warmer than WW without red_assist",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 10%\n        color_temperature: 2200 K\n",
        False,
        "warmer than the warm white LED needs red_assist",
    ),
    (
        "ultra dim_to_warm points same brightness",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 20%\n        color_temperature: 3000 K\n"
        "      - brightness: 20%\n        color_temperature: 2800 K\n",
        False,
        "two points with the same brightness",
    ),
    (
        "ultra dim_to_warm five points",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        + "".join(f"      - brightness: {b}%\n        color_temperature: 3000 K\n" for b in (90, 70, 50, 30, 10)),
        False,
        "at most 4",
    ),
    (
        "ultra dim_to_warm point colder than CW",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 50%\n        color_temperature: 8000 K\n",
        False,
        "colder than the cold white LED",
    ),
    (
        "ultra dim_to_warm point at 0 %",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      points:\n"
        "      - brightness: 0%\n        color_temperature: 3000 K\n",
        False,
        "brightness must be above 0 %",
    ),
    (
        "ultra dim_to_warm points and warm_color_temperature",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 2700 K\n      points:\n"
        "      - brightness: 20%\n        color_temperature: 3000 K\n",
        False,
        "points or warm_color_temperature, not both",
    ),
    (
        "ultra dim_to_warm warm end colder than CW",
        ULTRA + ULTRA_CCT + "    dim_to_warm:\n      warm_color_temperature: 7000 K\n",
        False,
        "must be warmer than cold_white_color_temperature",
    ),
    (
        "ultra per-channel min_power",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    min_power: {default: 0.1%, ch1: 0.5%}\n    zero_means_zero: true\n",
        True,
        None,
    ),
    ("ultra strip", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2"), True, None),
    ("ultra strip effect_glide false", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    effect_glide: false\n",
     True, None),
    ("ultra strip effect_glide bogus", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    effect_glide: maybe\n",
     False, "boolean"),
    (
        "ultra strip with a non-Ultra light",
        LEDC + ULTRA + STRIP_PIXELS
        + "  - platform: monochromatic\n    id: m\n    name: m\n    output: o1\n" + STRIP.format(lights="d1, m"),
        False,
        "'m' is not a dig2analog_ultra light",
    ),
    ("ultra strip light twice", ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d1"), False, "'d1' is listed twice"),
    (
        "ultra strip gamma_correct",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + "    gamma_correct: 2.8\n",
        False,
        "leave gamma_correct at 1.0",
    ),
    (
        "ultra partition over a strip without gamma_correct 1.0",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + PARTITION.format(gamma=""),
        False,
        "set gamma_correct: 1.0 on the partition",
    ),
    (
        "ultra partition over a strip",
        ULTRA + STRIP_PIXELS + STRIP.format(lights="d1, d2") + PARTITION.format(gamma="    gamma_correct: 1.0\n"),
        True,
        None,
    ),
    (
        "ultra light effect_glide",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1)
        + "    effect_glide: false\n    effects:\n      - flicker:\n      - strobe:\n      - ultra_breathe:\n",
        True,
        None,
    ),
    (
        "ultra white-only light",
        ULTRA + "light:\n  - platform: dig2analog_ultra\n    name: w\n    board: 0\n    white: 4\n",
        True,
        None,
    ),
    ("ultra three lines at rmt_symbols 192", lines(3), False, "need 9 x 64"),
    ("ultra three lines at rmt_symbols 128", lines(3, 128), True, None),
    ("ultra two lines on an esp32-s3 at 192", lines(2), False, "use_dma: true", "esp32-s3-devkitc-1"),
    ("ultra two lines on an esp32-s3 at 96", lines(2, 96), True, None, "esp32-s3-devkitc-1"),
    ("ultra update minimal", ULTRA + UPD_NET + UPD, True, None),
    (
        "ultra update all options",
        ULTRA + UPD_NET + UPD
        + "    source: quinled/some-firmware\n    channel: beta\n    public_keys: test\n    board_revision: 2\n"
        "    passes: 2\n    update_interval: 1h\n    channel_select:\n      name: fw channel\n"
        "    status:\n      name: fw status\n",
        True,
        None,
    ),
    (
        "ultra update own server without signature",
        ULTRA + UPD_NET + UPD + "    source: http://nas.local/fw/{channel}.json\n    verify_signature: false\n",
        True,
        None,
    ),
    (
        "ultra update unsigned from github",
        ULTRA + UPD_NET + UPD + "    verify_signature: false\n",
        False,
        "only with your own source",
    ),
    ("ultra update bad source", ULTRA + UPD_NET + UPD + "    source: not a repo\n", False, "OWNER/REPO on GitHub"),
    ("ultra update channel bogus", ULTRA + UPD_NET + UPD + "    channel: nightly\n", False, "Unknown value"),
    ("ultra update two per line", ULTRA + UPD_NET + UPD + UPD.replace("id: fw", "id: fw2").replace("name: fw", "name: fw2").replace("update:\n", ""), False, "one per line"),
    # the line is quiet for a second after an update session: signal_loss off / fade_off must outlast it
    (
        "ultra update with signal_loss fade_off 1s",
        ULTRA + "  signal_loss: fade_off\n  signal_loss_timeout: 1s\n" + UPD_NET + UPD,
        False,
        "must be at least 2s",
    ),
    (
        "ultra update with signal_loss off 2s",
        ULTRA + "  signal_loss: \"off\"\n  signal_loss_timeout: 2s\n" + UPD_NET + UPD,
        True,
        None,
    ),
    (
        "ultra update with signal_loss hold 500ms",
        ULTRA + "  signal_loss_timeout: 500ms\n" + UPD_NET + UPD,
        True,
        None,
    ),
    ("ultra update without http_request", ULTRA + "wifi:\n  ssid: x\n  password: xxxxxxxx\n" + UPD, False, "http_request"),
    (
        "ultra update actions",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - update.perform:\n          id: fw\n          force_update: true\n"
        "      - dig2analog_ultra.abort_update: fw\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n          lead_in: 45s\n          power_cycle: false\n",
        True,
        None,
    ),
    (
        "ultra update recover lead_in too long",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - dig2analog_ultra.recover:\n          id: fw\n          lead_in: 1h\n",
        False,
        "value must be at most 10min",
    ),
    ("ultra esp32-c2 rejected (no RMT)", ULTRA, False, "RMT", "esp32-c2-devkitm-1"),
]


def run_case(idx, name, body, expect_ok, needle, board="esp32dev"):
    WORK.mkdir(parents=True, exist_ok=True)
    path = WORK / f"case{idx:02d}.yaml"
    path.write_text(
        BASE.format(board=board, components=(ROOT / "components").as_posix()) + body,
        encoding="utf-8",
    )
    proc = subprocess.run(
        ESPHOME + ["config", str(path)], capture_output=True, text=True, encoding="utf-8", errors="replace"
    )
    out = proc.stdout + proc.stderr
    ok = proc.returncode == 0
    problems = []
    if ok != expect_ok:
        problems.append(f"expected {'pass' if expect_ok else 'fail'}, got rc={proc.returncode}")
    if needle:
        if needle.startswith("!"):
            if needle[1:] in out:
                problems.append(f"unexpected '{needle[1:]}' in output")
        elif needle not in out:
            problems.append(f"'{needle}' not in output")
    status = "ok  " if not problems else "FAIL"
    print(f"{status} {name}" + ("" if not problems else f": {'; '.join(problems)}"))
    if problems:
        print(textwrap.indent(out[-1500:], "     | "))
    return not problems


def check_group_budget() -> bool:
    """The codegen's group budget (__init__.py _max_group_commands): the commands one Home Assistant change to every
    light can give the busiest board, which the hub's start lead covers."""
    sys.path.insert(0, str(ROOT / "components"))
    from dig2analog_ultra import _max_group_commands as budget

    plain = [{"board": 0} for _ in range(5)]
    effects = [{"board": 0, "effects": [{"ultra_breathe": {}}]} for _ in range(5)]
    strobes = [{"board": 0, "effects": [{"ultra_strobe": {"colors": [{}] * 8}}], "power_on_scene": True}] * 5
    cases = [
        ("no lights", [], 1),
        ("5 plain lights: a SET each", plain, 5),
        ("5 effect lights: + an EFFECT each", effects, 10),
        ("5 strobes with 8 steps and power_on_scene", strobes, 55),
        ("the busiest board counts", plain + [{"board": 1, "effects": [{"ultra_breathe": {}}]}], 5),
    ]
    ok = True
    for name, lights, want in cases:
        got = budget(lights)
        miss = "" if got == want else f" (want {want})"
        print(f"{'ok  ' if got == want else 'FAIL'} group budget, {name}: {got}{miss}")
        ok &= got == want
    return ok


def main() -> int:
    filters = [f.lower() for f in sys.argv[1:]]
    cases = list(enumerate(CASES))
    if filters:
        cases = [(i, c) for i, c in cases if any(f in c[0].lower() for f in filters)]
    budget_ok = filters != [] or check_group_budget()
    results = [run_case(i, *case) for i, case in cases]
    failed = results.count(False)
    print(f"\n{len(results) - failed}/{len(results)} schema cases passed")
    return 1 if failed or not budget_ok else 0


if __name__ == "__main__":
    sys.exit(main())
