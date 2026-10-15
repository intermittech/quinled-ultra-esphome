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
  devices:
    - id: r1
      name: Port 1 R1
    - id: r2
      name: Port 1 R2
    - id: shelf_1
      name: Shelf 1
esp32:
  board: {board}
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {components}
    components: [dig2analog_ultra, dig2analog_ultra_strip, diffpower, dig2analog_ultra_parlio]
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


# Diff-Power receiver telemetry (the return channel): a UART for the answers, the line's telemetry
RX_UART = """\
uart:
  - id: rx
    rx_pin: GPIO17
    baud_rate: 1000000
    rx_buffer_size: 1024
"""

TELEMETRY = "  telemetry:\n    uart_id: rx\n"

RX_LIST = "    name: Port 1\n    receivers:\n      - device_id: r1\n      - device_id: r2\n"


# Diff-Power ports (diffpower): a port with a plain light, a time source and the sun for "During dark"
DP_PORT = """\
diffpower:
  - id: p1
    name: Port 1
    enable_pin: GPIO26
    detect_pin: GPIO27
"""
# two ports on an ESP32-P4 (the PARLIO cases' lanes on them)
P4_PORTS = """\
diffpower:
  - id: dp1
    name: Port 1 power
    enable_pin: GPIO45
    detect_pin: GPIO46
  - id: dp2
    name: Port 2 power
    enable_pin: GPIO47
    detect_pin: GPIO48
"""
DP_MONO = """\
output:
  - platform: ledc
    id: o1
    pin: GPIO25
light:
  - platform: monochromatic
    id: mono
    name: mono
    output: o1
"""
DP_STRIP = """\
light:
  - platform: esp32_rmt_led_strip
    id: strip
    name: strip
    pin: GPIO18
    num_leds: 30
    chipset: ws2812
    rgb_order: GRB
  - platform: partition
    id: part
    name: part
    segments:
      - id: strip
        from: 0
        to: 9
"""
DP_WAKE = (
    "button:\n  - platform: template\n    name: Wake\n    on_press:\n      - diffpower.wake:\n"
    "          lights: [{light}]\n"
)
DP_SUN = """\
wifi:
  ssid: x
  password: xxxxxxxx
time:
  - platform: sntp
    id: t
sun:
  id: s
  latitude: 52.1
  longitude: 5.1
"""


def tel_lines(n, uarts=None, names=None, devices=None, extra=""):
    """n lines with telemetry (hubs on GPIO16, 18, 19, 21), each on its own uart (rx pins 22, 23, 25, 26) unless
    uarts says which (an index per line); port names Port 1..n unless names; a port device per line if devices."""
    hub_pins, rx_pins = [16, 18, 19, 21], [22, 23, 25, 26]
    out = "uart:\n"
    for i in range(n):
        out += f"  - id: rx{i}\n    rx_pin: GPIO{rx_pins[i]}\n    baud_rate: 1000000\n    rx_buffer_size: 1024\n"
    out += "dig2analog_ultra:\n"
    for i in range(n):
        out += (f"  - id: u{i}\n    pin: GPIO{hub_pins[i]}\n    num_boards: 1\n{extra}    telemetry:\n"
                f"      uart_id: rx{uarts[i] if uarts else i}\n      name: {names[i] if names else f'Port {i + 1}'}\n"
                f"      receivers: 1\n")
        if devices:
            out += f"      device_id: {devices[i]}\n"
    return out


def lines(n, symbols=None):
    out = "dig2analog_ultra:\n"
    for i in range(n):
        out += f"  - id: u{i}\n    pin: GPIO{16 + i}\n    num_boards: 1\n"
        if symbols:
            out += f"    rmt_symbols: {symbols}\n"
    return out


def parlio(lanes=16, pins=None, extra_bus="", hub_extra=None, uarts=4, telemetry=True, re=True):
    """dig2analog_ultra_parlio on an ESP32-P4: a bus of `pins` data pins (GPIO 0..), `lanes` hubs on lanes 0.. with 1
    board each; with telemetry 4 lanes share each of `uarts` UARTs (rx GPIO 16..), each lane a direction pin (GPIO 20..)
    and a receiver enable pin (GPIO 43..) unless re is False. hub_extra: {lane: extra YAML lines}."""
    pins = lanes if pins is None else pins
    de = [20, 21, 22, 23, 26, 27, 28, 29, 30, 31, 32, 33, 39, 40, 41, 42]
    rx_en = [43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 34, 35, 36, 37]
    out = "dig2analog_ultra_parlio:\n  - id: ports\n    pins: [" + ", ".join(f"GPIO{i}" for i in range(pins)) + "]\n"
    out += extra_bus
    if telemetry:
        out += "uart:\n"
        for u in range(uarts):
            out += f"  - id: g{u}\n    rx_pin: GPIO{16 + u}\n    baud_rate: 1000000\n    rx_buffer_size: 1024\n"
    out += "dig2analog_ultra:\n"
    for k in range(lanes):
        out += f"  - id: p{k}\n    parlio_id: ports\n    lane: {k}\n    num_boards: 1\n"
        out += (hub_extra or {}).get(k, "")
        if telemetry:
            out += (f"    telemetry:\n      uart_id: g{k // 4 % uarts}\n      name: Port {k + 1}\n      receivers: 1\n"
                    f"      diagnostics: none\n      direction_pin: GPIO{de[k]}\n")
            if re:
                strap = "        ignore_strapping_warning: true\n" if rx_en[k] in (34, 35, 36, 37) else ""
                out += f"      receiver_enable_pin:\n        number: GPIO{rx_en[k]}\n{strap}"
    return out


P4 = "esp32-p4_r3"
PARLIO_RMT_LINES = (
    "  - id: m0\n    pin: GPIO24\n    num_boards: 1\n    rmt_symbols: 48\n"
    "  - id: m1\n    pin: GPIO25\n    num_boards: 1\n    rmt_symbols: 48\n"
    "  - id: m2\n    pin: GPIO38\n    num_boards: 1\n    rmt_symbols: 48\n"
)


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
    # protocol 0.2.5: signal loss to the power-on scene; the trips' retry policy (CONFIG 24)
    ("ultra signal_loss scene", ULTRA + "  signal_loss: scene\n", True, None),
    (
        "ultra signal_loss scene with a slow refresh",
        ULTRA + "  refresh_interval: 2s\n  signal_loss: scene\n  signal_loss_timeout: 500ms\n",
        False,
        "with signal_loss: scene",
    ),
    ("ultra trip retries", ULTRA + "  trip_retries: 3\n  trip_retry_interval: 5s\n", True, None),
    ("ultra trip retries forever", ULTRA + "  trip_retries: forever\n", True, None),
    ("ultra trip retries never", ULTRA + "  trip_retries: 0\n", True, None),
    ("ultra trip retry interval alone", ULTRA + "  trip_retry_interval: 30s\n", True, None),
    ("ultra trip retries 255", ULTRA + "  trip_retries: 255\n", False, "254"),
    ("ultra trip retries bogus", ULTRA + "  trip_retries: always_on\n", False, "trip_retries"),
    ("ultra trip retry interval 0", ULTRA + "  trip_retry_interval: 0s\n", False, "trip_retry_interval"),
    ("ultra trip retry interval 5 min", ULTRA + "  trip_retry_interval: 5min\n", False, "trip_retry_interval"),
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
        "    passes: 2\n    auto_retry: 3\n    update_interval: 1h\n    channel_select:\n      name: fw channel\n"
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
    # auto_retry: sessions again for boards that missed one (0: off)
    ("ultra update auto_retry off", ULTRA + UPD_NET + UPD + "    auto_retry: 0\n", True, None),
    ("ultra update auto_retry 6", ULTRA + UPD_NET + UPD + "    auto_retry: 6\n", False, "auto_retry"),
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
    # Diff-Power receiver telemetry
    ("ultra telemetry minimal (receivers: every board, on the hub's device)", ULTRA + TELEMETRY + RX_UART, True, None),
    # Diff-Power ports
    ("diffpower port minimal", DP_PORT.replace("    detect_pin: GPIO27\n", ""), True, None),
    (
        "diffpower port: a plain light with its own turn-off period, options",
        DP_PORT + "    standby_after: 5min\n    hotplug_delay: 2s\n    boot_margin: 300ms\n    notice_time: 250ms\n"
        "    min_off_time: 2s\n    min_on_time: 5min\n    power_mode: always_on\n"
        "    lights:\n      - id: mono\n        standby_after: 30min\n    standby_cycles: false\n"
        "    standby_hours:\n      name: Port 1 standby\n    detect_drops: false\n" + DP_MONO,
        True,
        None,
    ),
    ("diffpower during_dark without a sun", DP_PORT + "    power_mode: during_dark\n", False, "needs a sun_id"),
    (
        "diffpower during_dark with the sun",
        DP_PORT + "    power_mode: during_dark\n    sun_id: s\n    dark_before: 30min\n    dark_after: 2h\n"
        "    time_unknown: standby\n" + DP_SUN,
        True,
        None,
    ),
    ("diffpower time_unknown bogus", DP_PORT + "    time_unknown: maybe\n", False, "time_unknown"),
    ("diffpower notice_time too short", DP_PORT + "    notice_time: 100ms\n", False, "notice_time"),
    # longer than the cap (and the receiver's 2 s) refused; 1 s the longest
    ("diffpower notice_time 2 s", DP_PORT + "    notice_time: 2s\n", False, "notice_time"),
    ("diffpower notice_time 1 s", DP_PORT + "    notice_time: 1s\n", True, None),
    ("diffpower power mode bogus", DP_PORT + "    power_mode: sometimes\n", False, "power_mode"),
    ("diffpower turn-off period too long", DP_PORT + "    standby_after: 2d\n", False, "standby_after"),
    ("diffpower unknown light", DP_PORT + "    lights: [nope]\n", False, "Couldn't find ID 'nope'"),
    # the off time at least 1 s; a light's turn-off period at most 24 h; wake targets on a port
    ("diffpower min_off_time 0", DP_PORT + "    min_off_time: 0ms\n", False, "min_off_time"),
    ("diffpower min_off_time 500 ms", DP_PORT + "    min_off_time: 500ms\n", False, "min_off_time"),
    (
        "diffpower light turn-off period 60 days",
        DP_PORT + "    lights:\n      - id: mono\n        standby_after: 60d\n" + DP_MONO,
        False,
        "standby_after",
    ),
    (
        "diffpower.wake: a light on no port",
        DP_PORT + DP_MONO + DP_WAKE.format(light="mono"),
        False,
        "light 'mono' is on no Diff-Power port",
    ),
    (
        "diffpower.wake: a light of an Ultra line on a port",
        DP_PORT + ULTRA + "  port: p1\n" + "light:\n  - platform: dig2analog_ultra\n    id: l1\n    name: l1\n"
        "    board: 0\n    channel: 1\n" + DP_WAKE.format(light="l1"),
        True,
        None,
    ),
    (
        "diffpower.wake: a light of an Ultra line without a port",
        DP_PORT + ULTRA + "light:\n  - platform: dig2analog_ultra\n    id: l1\n    name: l1\n"
        "    board: 0\n    channel: 1\n" + DP_WAKE.format(light="l1"),
        False,
        "light 'l1' is on no Diff-Power port",
    ),
    # a partition of a strip on the port is the port's too (its pixels)
    (
        "diffpower: a strip on a port, wake by its partition",
        DP_PORT + "    lights: [strip]\n" + DP_STRIP + DP_WAKE.format(light="part"),
        True,
        None,
    ),
    # a partition of a partition of the strip on the port is the port's too
    (
        "diffpower: a strip on a port, wake by a nested partition",
        DP_PORT + "    lights: [strip]\n" + DP_STRIP
        + "  - platform: partition\n    id: outer\n    name: outer\n    segments:\n      - id: part\n"
        "        from: 0\n        to: 4\n" + DP_WAKE.format(light="outer"),
        True,
        None,
    ),
    # two ports with one name (their entities off): accepted, their counters kept apart by id
    (
        "diffpower two ports with one name",
        DP_PORT + "    status: false\n    problem: false\n    standby_cycles: false\n    standby_hours: false\n"
        "    detect_drops: false\n    power_mode_select: false\n    wake: false\n"
        "  - id: p2\n    name: Port 1\n    enable_pin: GPIO32\n    detect_pin: GPIO33\n    status: false\n"
        "    problem: false\n    standby_cycles: false\n    standby_hours: false\n    detect_drops: false\n"
        "    power_mode_select: false\n    wake: false\n",
        True,
        None,
    ),
    (
        "ultra line on a port, a light with its own turn-off period",
        DP_PORT + ULTRA + "  port: p1\n" + "light:\n  - platform: dig2analog_ultra\n    name: l1\n    board: 0\n"
        "    channel: 1\n    standby_after: 1h\n",
        True,
        None,
    ),
    (
        "ultra line on a port and a power_supply",
        DP_PORT + ULTRA + "  port: p1\n  power_supply: psu\n" + PSU.format(enable="20ms"),
        False,
        "port and power_supply both power the line",
    ),
    (
        "two ultra lines on one port",
        DP_PORT + "dig2analog_ultra:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    port: p1\n"
        "  - id: b\n    pin: GPIO17\n    num_boards: 1\n    port: p1\n",
        False,
        "are both on port 'p1'",
    ),
    (
        "diffpower.wake: ports and lights",
        DP_PORT + "    lights: [mono]\n" + DP_MONO + "binary_sensor:\n  - platform: gpio\n    pin: GPIO33\n"
        "    name: motion\n    on_press:\n      - diffpower.wake:\n          ports: [p1]\n          timeout: 3min\n"
        "      - diffpower.wake:\n          lights: [mono]\n",
        True,
        None,
    ),
    (
        "diffpower.wake without ports or lights",
        DP_PORT + "binary_sensor:\n  - platform: gpio\n    pin: GPIO33\n    name: motion\n    on_press:\n"
        "      - diffpower.wake:\n          timeout: 3min\n",
        False,
        "give ports, lights or both",
    ),
    # several lines with telemetry: a UART, a port name and a port device each
    ("ultra telemetry on 2 lines, a UART each", tel_lines(2), True, None),
    ("ultra telemetry 2 lines on one UART", tel_lines(2, uarts=[0, 0]), False, "use the same uart 'rx0'"),
    # (ESPHome's own check: entity names unique per device)
    ("ultra telemetry 2 lines, one port name", tel_lines(2, names=["Port 1", "port 1"]), False, "Duplicate"),
    ("ultra telemetry 2 lines, one port device", tel_lines(2, devices=["r1", "r1"]), False, "on device 'r1'"),
    ("ultra telemetry 2 lines, a port device each", tel_lines(2, devices=["r1", "r2"]), True, None),
    (
        "ultra telemetry on 4 lines of an esp32-p4 (3 x 48 symbols + 1 with DMA)",
        tel_lines(4, extra="    rmt_symbols: 48\n").replace(
            "  - id: u3\n    pin: GPIO21\n    num_boards: 1\n    rmt_symbols: 48\n",
            "  - id: u3\n    pin: GPIO21\n    num_boards: 1\n    use_dma: true\n",
        ),
        True,
        None,
        "esp32-p4_r3",
    ),
    ("ultra telemetry on 4 lines of an esp32-p4 at 192 symbols", tel_lines(4), False, "the RMT of the ESP32P4", "esp32-p4_r3"),
    # the DMA line takes one channel, the others their blocks: 2 x 48 + 1 x 96 + DMA = 5 of 4
    (
        "ultra telemetry on 4 lines of an esp32-p4, one at 96 symbols + 1 with DMA",
        tel_lines(4, extra="    rmt_symbols: 48\n")
        .replace("  - id: u2\n    pin: GPIO19\n    num_boards: 1\n    rmt_symbols: 48\n",
                 "  - id: u2\n    pin: GPIO19\n    num_boards: 1\n    rmt_symbols: 96\n")
        .replace("  - id: u3\n    pin: GPIO21\n    num_boards: 1\n    rmt_symbols: 48\n",
                 "  - id: u3\n    pin: GPIO21\n    num_boards: 1\n    use_dma: true\n"),
        False,
        "need 5 x 48",
        "esp32-p4_r3",
    ),
    (
        "ultra 2 lines with DMA on an esp32-p4",
        tel_lines(2, extra="    use_dma: true\n"),
        False,
        "only one RMT transmitter of the ESP32P4 can use DMA",
        "esp32-p4_r3",
    ),
    (
        "ultra telemetry full: a device per receiver, renames, lights on devices",
        ULTRA + TELEMETRY + RX_LIST + "        power:\n          name: Power in\n    direction_pin: GPIO4\n"
        "    diagnostics: full\n    poll_intervals:\n      status: 2s\n      energy: 5min\n" + RX_UART
        + "light:\n  - platform: dig2analog_ultra\n    name: l1\n    board: 1\n    channel: 1\n    device_id: shelf_1\n"
        "  - platform: dig2analog_ultra\n    name: l2\n    board: 1\n    channel: 2\n",
        True,
        None,
    ),
    (
        "ultra telemetry receivers as device ids",
        ULTRA + TELEMETRY + "    receivers: [r1, r2]\n" + RX_UART,
        True,
        None,
    ),
    ("ultra telemetry 500 kbaud", ULTRA + TELEMETRY + RX_UART.replace("1000000", "500000"), True, None),
    (
        "ultra telemetry baud rate",
        ULTRA + TELEMETRY + RX_UART.replace("1000000", "115200"),
        False,
        "isn't a rate the receivers answer at",
    ),
    (
        "ultra telemetry rx buffer",
        ULTRA + TELEMETRY + RX_UART.replace("1024", "256"),
        False,
        "rx_buffer_size must be at least 1024",
    ),
    (
        "ultra telemetry uart without rx",
        ULTRA + TELEMETRY + RX_UART.replace("rx_pin: GPIO17", "tx_pin: GPIO17"),
        False,
        "needs an rx_pin",
    ),
    (
        "ultra telemetry parity",
        ULTRA + TELEMETRY + RX_UART + "    parity: EVEN\n",
        False,
        "the answers are 8N1",
    ),
    (
        "ultra telemetry receivers > num_boards",
        ULTRA + TELEMETRY + "    receivers: 3\n" + RX_UART,
        False,
        "3 receivers can't be more than num_boards (2)",
    ),
    (
        "ultra telemetry status interval",
        ULTRA + TELEMETRY + "    poll_intervals:\n      status: 100ms\n" + RX_UART,
        False,
        "status",
    ),
    (
        "ultra telemetry 0.2.5: lifetime / diag intervals, its entities renamed",
        ULTRA + TELEMETRY + RX_LIST + "        led_supply:\n          name: LED volts\n        lit_hours:\n"
        "          name: Burn time\n    port_peak_load:\n      name: Port 1 peaks\n    poll_intervals:\n"
        "      lifetime: 1h\n      diag: 2min\n" + RX_UART,
        True,
        None,
    ),
    (
        "ultra telemetry diag interval",
        ULTRA + TELEMETRY + "    poll_intervals:\n      diag: 10s\n" + RX_UART,
        False,
        "diag",
    ),
    (
        "ultra telemetry diagnostics bogus",
        ULTRA + TELEMETRY + "    diagnostics: some\n" + RX_UART,
        False,
        "diagnostics",
    ),
    (
        "ultra telemetry unknown device",
        ULTRA + TELEMETRY + "    receivers:\n      - device_id: nowhere\n" + RX_UART,
        False,
        "nowhere",
    ),
    (
        "ultra telemetry two ports with the same name, no devices",
        "dig2analog_ultra:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
        "  - id: b\n    pin: GPIO17\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
        + RX_UART.replace("GPIO17", "GPIO18"),
        False,
        "Duplicate",
    ),
    (
        "ultra telemetry two ports with names",
        "dig2analog_ultra:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
        "      name: Port 1\n  - id: b\n    pin: GPIO17\n    num_boards: 1\n    telemetry:\n      uart_id: rx2\n"
        "      name: Port 2\n" + RX_UART.replace("GPIO17", "GPIO18")
        + "  - id: rx2\n    rx_pin: GPIO19\n    baud_rate: 1000000\n    rx_buffer_size: 1024\n",
        True,
        None,
    ),
    (
        "ultra telemetry port load options",
        ULTRA + TELEMETRY + "    supply_voltage: 55V\n    port_limit: 2.25A\n    load_warning: 85%\n"
        "    load_problem: 95%\n    load_problem_time: 1min\n" + RX_UART,
        True,
        None,
    ),
    (
        "ultra telemetry supply voltage too high",
        ULTRA + TELEMETRY + "    supply_voltage: 70V\n" + RX_UART,
        False,
        "supply_voltage",
    ),
    (
        "ultra telemetry port limit too low",
        ULTRA + TELEMETRY + "    port_limit: 50mA\n" + RX_UART,
        False,
        "port_limit",
    ),
    (
        "ultra diagnostics sensors (the line)",
        ULTRA + "sensor:\n  - platform: dig2analog_ultra\n    queue_peak:\n      name: q\n",
        True,
        None,
    ),
    # several lines on one UART: each with a receiver_enable_pin (one window at a time)
    (
        "ultra telemetry 2 lines on one UART with receiver enable pins",
        tel_lines(2, uarts=[0, 0]).replace("      receivers: 1\n", "      receivers: 1\n      receiver_enable_pin: GPIO{}\n").format(4, 5),
        True,
        None,
    ),
    (
        "ultra telemetry 3 lines on one UART, one without a receiver enable pin",
        tel_lines(3, uarts=[0, 0, 0], extra="    rmt_symbols: 128\n").replace("      receivers: 1\n", "      receivers: 1\n      receiver_enable_pin: GPIO{}\n", 2).format(4, 5),
        False,
        "'u2' has none",
    ),
    (
        "ultra telemetry receiver enable pin on a line of its own (allowed, not needed)",
        ULTRA + TELEMETRY + "    receiver_enable_pin:\n      number: GPIO4\n      inverted: true\n" + RX_UART,
        True,
        None,
    ),
    # Home Assistant (quinled-ultra-homeassistant esphome-requirements): the update rate, the live twins, the Port text,
    # the Identify button, the controller's totals, DE and /RE tied on a shared UART
    ("ultra telemetry update_rate normal", ULTRA + TELEMETRY + "    update_rate: normal\n" + RX_UART, True, None),
    ("ultra telemetry update_rate bogus", ULTRA + TELEMETRY + "    update_rate: ludicrous\n" + RX_UART, False,
     "Unknown value"),
    ("ultra telemetry live twins on devices", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Live power"),
    ("ultra telemetry Port text on a receiver's device", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Port\n"),
    ("ultra telemetry Identify button", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Identify"),
    ("ultra telemetry no Port text without a device", ULTRA + TELEMETRY + RX_UART, True, "!R1 port"),
    ("ultra telemetry port live power and energy", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Port 1 energy"),
    ("ultra telemetry line buttons (disabled)", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Port 1 calibrate current"),
    (
        "ultra telemetry 2 lines on one UART, DE and /RE tied",
        tel_lines(2, uarts=[0, 0]).replace(
            "      receivers: 1\n", "      receivers: 1\n      direction_pin: GPIO{}\n      receiver_enable_pin: direction_pin\n"
        ).format(4, 5),
        True,
        None,
    ),
    (
        "ultra telemetry DE and /RE tied without a direction pin",
        ULTRA + TELEMETRY + "    receiver_enable_pin: direction_pin\n" + RX_UART,
        False,
        "needs a direction_pin",
    ),
    ("diffpower on its own device: names without the port's", DP_PORT + "    device_id: r1\n", True, "name: Power state"),
    ("diffpower on its own device: a Wake button", DP_PORT + "    device_id: r1\n", True, "name: Wake\n"),
    ("diffpower on the controller: '<port> wake'", DP_PORT, True, "name: Port 1 wake"),
    ("diffpower wake_timeout", DP_PORT + "    wake_timeout: 30min\n", True, None),
    ("diffpower wake_timeout 2 days", DP_PORT + "    wake_timeout: 48h\n", False, "wake_timeout"),
    # PARLIO: up to 16 lanes on an ESP32-P4, telemetry on 4 UARTs shared by 4 lanes each
    ("parlio 16 lanes, 4 UARTs x 4 lanes with receiver enable pins", parlio(), True, None, P4),
    ("parlio lanes on Diff-Power ports",
     parlio(lanes=2, telemetry=False, hub_extra={0: "    port: dp1\n", 1: "    port: dp2\n"}) + P4_PORTS, True, None, P4),
    ("parlio two lanes on one Diff-Power port",
     parlio(lanes=2, telemetry=False, hub_extra={0: "    port: dp1\n", 1: "    port: dp1\n"}) + P4_PORTS, False,
     "are both on port 'dp1'", P4),
    ("parlio 16 lanes without telemetry", parlio(telemetry=False), True, None, P4),
    ("parlio 3 lanes (8-bit samples)", parlio(lanes=3, telemetry=False), True, None, P4),
    ("parlio 2 lanes of a 4-pin bus (2 idle)", parlio(lanes=2, pins=4, telemetry=False), True, None, P4),
    ("parlio two buses (the P4 has one PARLIO TX unit)",
     parlio(lanes=1, telemetry=False).replace("dig2analog_ultra:\n",
                                              "  - id: more\n    pins: [GPIO26]\ndig2analog_ultra:\n"),
     False, "the ESP32-P4 has one PARLIO TX unit", P4),
    ("parlio 4 lanes sharing a UART without receiver enable pins", parlio(lanes=4, uarts=1, re=False), False,
     "use the same uart 'g0'", P4),
    ("parlio on an esp32", parlio(lanes=2, telemetry=False), False, "dig2analog_ultra_parlio (PARLIO TX, 16 lines)"),
    ("parlio on an esp32-s3", parlio(lanes=2, telemetry=False), False, "dig2analog_ultra_parlio",
     "esp32-s3-devkitc-1"),
    ("parlio two hubs on one lane", parlio(lanes=2, telemetry=False).replace("lane: 1\n", "lane: 0\n"), False,
     "are both on lane 0 of 'ports'", P4),
    ("parlio a lane beyond the pins", parlio(lanes=3, pins=2, telemetry=False), False, "has 2 pins (lanes 0-1)", P4),
    ("parlio 17 pins", parlio(lanes=1, pins=16, telemetry=False).replace("GPIO15]", "GPIO15, GPIO26]"), False,
     "1-16 data pins", P4),
    ("parlio a hub with a pin and a parlio_id", parlio(lanes=1, telemetry=False, hub_extra={0: "    pin: GPIO26\n"}),
     False, "give the line a pin (RMT), or a parlio_id and lane", P4),
    ("parlio a lane without parlio_id",
     parlio(lanes=1, telemetry=False).replace("    parlio_id: ports\n", "    pin: GPIO26\n"), False,
     "lane goes with parlio_id", P4),
    ("parlio use_dma on a lane", parlio(lanes=1, telemetry=False, hub_extra={0: "    use_dma: true\n"}), False,
     "use_dma is an RMT option", P4),
    ("parlio an unknown bus", parlio(lanes=1, telemetry=False).replace("parlio_id: ports", "parlio_id: nowhere"),
     False, "nowhere", P4),
    ("parlio one data pin twice", parlio(lanes=2, telemetry=False).replace("GPIO1]", "GPIO0]"), False,
     "Pin 0 is used in multiple places", P4),
    ("parlio sample_rate 10 MHz ('0' high 300 ns)", parlio(lanes=2, telemetry=False, extra_bus="    sample_rate: 10MHz\n"),
     True, None, P4),
    ("parlio sample_rate 40 MHz", parlio(lanes=2, telemetry=False, extra_bus="    sample_rate: 40MHz\n"), True, None,
     P4),
    ("parlio sample_rate 16 MHz (not whole ns)", parlio(lanes=2, telemetry=False, extra_bus="    sample_rate: 16MHz\n"),
     False, "sample_rate must divide the PARLIO clock", P4),
    ("parlio sample_rate 5 MHz ('0' high 200 ns)", parlio(lanes=2, telemetry=False, extra_bus="    sample_rate: 5MHz\n"),
     False, "lane 0: bit0_high becomes 200 ns at 5 MHz", P4),
    ("parlio a lane's bit1_high 2.1 us", parlio(lanes=2, telemetry=False, hub_extra={1: "    bit1_high: 2100ns\n"}),
     False, "lane 1: bit1_high becomes 2100 ns", P4),
    ("parlio lanes with their own high times", parlio(lanes=2, telemetry=False, hub_extra={1: "    bit0_high: 300ns\n"
                                                                                           "    bit1_high: 800ns\n"}),
     True, None, P4),
    ("parlio buffer in PSRAM, 2 ms", parlio(lanes=2, telemetry=False, extra_bus="    psram: true\n    buffer_time: 2ms\n"),
     True, None, P4),
    ("parlio buffer_time 20 ms", parlio(lanes=2, telemetry=False, extra_bus="    buffer_time: 20ms\n"), False,
     "buffer_time", P4),
    # a buffer must be with the DMA two descriptors (2 x 4095 B) before the one playing ends, and leave the render time
    ("parlio 8 lanes at 10 MHz, buffer_time 1 ms (shorter than the DMA's look-ahead + the render)",
     parlio(lanes=8, telemetry=False, extra_bus="    sample_rate: 10MHz\n    buffer_time: 1ms\n"), False,
     "buffer_time 1000 us is too short at 10 MHz with 8-bit samples", P4),
    ("parlio 8 lanes at 10 MHz, buffer_time auto (longer than 1 ms)",
     parlio(lanes=8, telemetry=False, extra_bus="    sample_rate: 10MHz\n"), True, None, P4),
    ("parlio 16 lanes, buffer_time 600 us", parlio(telemetry=False, extra_bus="    buffer_time: 600us\n"), False,
     "at least 655 us", P4),
    # lanes don't use the RMT: 16 lanes and the P4's 4 RMT lines (3 x 48 + DMA) fit; the RMT lines still add up
    ("parlio 16 lanes and 4 RMT lines (3 x 48 symbols + DMA)",
     parlio(telemetry=False) + PARLIO_RMT_LINES + "  - id: m3\n    pin: GPIO39\n    num_boards: 1\n    use_dma: true\n",
     True, None, P4),
    ("parlio 16 lanes and 5 RMT lines at 48 symbols",
     parlio(telemetry=False) + PARLIO_RMT_LINES + "  - id: m3\n    pin: GPIO39\n    num_boards: 1\n    rmt_symbols: 48\n"
     "  - id: m4\n    pin: GPIO40\n    num_boards: 1\n    rmt_symbols: 48\n", False, "need 5 x 48", P4),
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
