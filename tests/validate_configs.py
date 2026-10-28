"""Schema tests: run `esphome config` on small configs and check pass/fail and messages.

Usage: .venv/Scripts/python tests/validate_configs.py [name filter ...]
       (with filters, only the cases whose name contains one of them run)
"""

from pathlib import Path
import re
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
    components: [ultra_line, dig2analog_plus, ur_analog_m, ultra_analog_strip, diffpower, ultra_parlio, ur_digital_m, ur_digital_m_stream]
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
ultra_line:
  id: u
  pin: GPIO16
  num_boards: 2
"""

# a normal Ultra line of three boards (chain mode: a Diff-Power Ultra Receiver Digital-Medium at any position)
ULTRA_CH = ULTRA.replace("num_boards: 2", "num_boards: 3")

# a digital port: one Diff-Power-UR-Digital-M in a pixel strip's data path
ULTRA_DP = """\
ultra_line:
  id: u
  pin: GPIO16
  num_boards: 1
  digital:
"""

ULTRA_RGBWW = """\
light:
  - platform: dig2analog_plus
    name: rgbww
    board: 1
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
  - platform: dig2analog_plus
    name: cct
    board: 1
    warm_white: 1
    cold_white: 2
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
"""

ULTRA_MONO = """\
  - platform: dig2analog_plus
    name: {name}
    board: {board}
    channel: {channel}
"""

# the same light on a Diff-Power-UR-Analog-M (a receiver: it answers on a line with telemetry)
AM_MONO = ULTRA_MONO.replace("platform: dig2analog_plus", "platform: ur_analog_m")

MONO_CANDLE = """\
  - platform: dig2analog_plus
    name: niche
    board: 1
    channel: 1
    effects:
      - ultra_candle:
"""

STRIP_PIXELS = """\
light:
  - platform: dig2analog_plus
    id: d1
    name: d1
    board: 1
    channel: 1
  - platform: dig2analog_plus
    id: d2
    name: d2
    board: 1
    channel: 2
"""

STRIP = """\
  - platform: ultra_analog_strip
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
# "During dark" from Home Assistant: the API and its time
DP_HA = """\
wifi:
  ssid: x
  password: xxxxxxxx
api:
time:
  - platform: homeassistant
    id: ha_time
"""


def tel_lines(n, uarts=None, names=None, devices=None, extra=""):
    """n lines with telemetry (hubs on GPIO16, 18, 19, 21), each on its own uart (rx pins 22, 23, 25, 26) unless
    uarts says which (an index per line); port names Port 1..n unless names; a port device per line if devices."""
    hub_pins, rx_pins = [16, 18, 19, 21], [22, 23, 25, 26]
    out = "uart:\n"
    for i in range(n):
        out += f"  - id: rx{i}\n    rx_pin: GPIO{rx_pins[i]}\n    baud_rate: 1000000\n    rx_buffer_size: 1024\n"
    out += "ultra_line:\n"
    for i in range(n):
        out += (f"  - id: u{i}\n    pin: GPIO{hub_pins[i]}\n    num_boards: 1\n{extra}    telemetry:\n"
                f"      uart_id: rx{uarts[i] if uarts else i}\n      name: {names[i] if names else f'Port {i + 1}'}\n"
                f"      receivers: 1\n")
        if devices:
            out += f"      device_id: {devices[i]}\n"
    return out


def ram_lines(n, receivers, diagnostics="full"):
    """n lines of `receivers` receivers each (a light per receiver), each with telemetry on its own UART."""
    hub_pins, rx_pins = [16, 18, 19, 21], [22, 23, 25, 26]
    out = "uart:\n"
    for i in range(n):
        out += f"  - id: rx{i}\n    rx_pin: GPIO{rx_pins[i]}\n    baud_rate: 1000000\n    rx_buffer_size: 2048\n"
    out += "ultra_line:\n"
    for i in range(n):
        out += (f"  - id: u{i}\n    pin: GPIO{hub_pins[i]}\n    num_boards: {receivers}\n    rmt_symbols: 64\n"
                f"    telemetry:\n      uart_id: rx{i}\n      name: Port {i + 1}\n      receivers: {receivers}\n"
                f"      diagnostics: {diagnostics}\n")
    out += "light:\n"
    for i in range(n):
        for b in range(receivers):
            out += (f"  - platform: ur_analog_m\n    ultra_line_id: u{i}\n    name: l{i}_{b}\n"
                    f"    board: {b + 1}\n    channel: 1\n")
    return out


def lines(n, symbols=None):
    out = "ultra_line:\n"
    for i in range(n):
        out += f"  - id: u{i}\n    pin: GPIO{16 + i}\n    num_boards: 1\n"
        if symbols:
            out += f"    rmt_symbols: {symbols}\n"
    return out


def parlio(lanes=16, pins=None, extra_bus="", hub_extra=None, uarts=4, telemetry=True, re=True):
    """ultra_parlio on an ESP32-P4: a bus of `pins` data pins (GPIO 0..), `lanes` hubs on lanes 0.. with 1
    board each; with telemetry 4 lanes share each of `uarts` UARTs (rx GPIO 16..), each lane a direction pin (GPIO 20..)
    and a receiver enable pin (GPIO 43..) unless re is False. hub_extra: {lane: extra YAML lines}."""
    pins = lanes if pins is None else pins
    de = [20, 21, 22, 23, 26, 27, 28, 29, 30, 31, 32, 33, 39, 40, 41, 42]
    rx_en = [43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 34, 35, 36, 37]
    out = "ultra_parlio:\n  - id: ports\n    pins: [" + ", ".join(f"GPIO{i}" for i in range(pins)) + "]\n"
    out += extra_bus
    if telemetry:
        out += "uart:\n"
        for u in range(uarts):
            out += f"  - id: g{u}\n    rx_pin: GPIO{16 + u}\n    baud_rate: 1000000\n    rx_buffer_size: 1024\n"
    out += "ultra_line:\n"
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
UPD = "update:\n  - platform: ultra_line\n    id: fw\n    name: fw\n    ultra_line_id: u\n"
# an inventory item: a V203 board's protected boot-info export (tests/fixtures/boot_info_v203.bin)
BOOT_INFO = (Path(__file__).resolve().parent / "fixtures" / "boot_info_v203.bin").as_posix()
UPD_INV = "    inventory:\n"


def inv_item(board, extra=""):
    return f"      - board: {board}\n        boot_info: {BOOT_INFO}\n" + extra


def complete_pin(boards, mask=15, skip=()):
    """a pin's complete all-Ultra inventory (after the hub's keys): line_rate facts and a descriptors: file per board
    1..boards but the skip ones (the schema only checks that the file exists; its bytes are read when the
    configuration compiles)"""
    return (f"  line_rate:\n    inventory_complete: true\n    bootloader_profile_mask: {mask}\n  descriptors:\n" +
            "".join(f"    - board: {b}\n      file: {BOOT_INFO}\n" for b in range(1, boards + 1) if b not in skip))

# Diff-Power-UR-Digital-M receivers' local pixel effects (ur_digital_m, the PIXEL_FX of protocol 0.3)
UR_DIGITAL_M = """\
light:
  - platform: ur_digital_m
    name: strip
    board: 1
    chip: ws2812b
    num_leds: 60
"""
UR_DIGITAL_M_LIGHT = """\
  - platform: ur_digital_m
    name: {name}
    board: {board}
    segment: {segment}
    chip: {chip}
    num_leds: {leds}
{extra}"""
UR_DIGITAL_M_STREAM = """\
  - platform: esp32_rmt_led_strip
    id: streamer
    name: streamer
    pin: GPIO18
    num_leds: 30
    chipset: ws2812
    rgb_order: GRB
"""


UR_DIGITAL_M_STREAM_LIGHT = """\
  - platform: ur_digital_m_stream
    id: streamer
    name: stream
    ultra_line_id: u
    num_leds: 30
    chipset: ws2812
"""


def ur_digital_m_light(name="s", board=1, segment=0, chip="ws2812b", leds=60, extra=""):
    return UR_DIGITAL_M_LIGHT.format(name=name, board=board, segment=segment, chip=chip, leds=leds, extra=extra)


def ur_digital_m_effects(*names_and_ids, only=False):
    body = "".join(f"      - ur_digital_m_fx:\n          name: {n}\n          fx: {i}\n" for n, i in names_and_ids)
    return ("    catalogue_effects: false\n" if only else "") + "    effects:\n" + body


UR_DIGITAL_M_ACTION = (
    "button:\n  - platform: template\n    name: B\n    on_press:\n      - ur_digital_m.effect_params:\n"
    "          id: {id}\n{fields}"
)


# (name, yaml body, expect_ok, substring expected in output or None, board)
CASES = [
    ("rate request diagnostic default disabled", ULTRA + TELEMETRY + RX_LIST + RX_UART,
     True, "Lower line rate requested"),
    ("rate request explicit with diagnostics none", ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST.replace(
        "      - device_id: r1\n", "      - device_id: r1\n        rate_request: {}\n") + RX_UART,
     True, "entity_category: diagnostic"),
    ("rate request can be disabled", ULTRA + TELEMETRY + RX_LIST.replace(
        "      - device_id: r1\n", "      - device_id: r1\n        rate_request: false\n") + RX_UART,
     True, None),
    ("output voltage low explicit diagnostic", ULTRA + TELEMETRY + RX_LIST.replace(
        "      - device_id: r1\n", "      - device_id: r1\n        output_voltage_low:\n          name: Output voltage low / fuse\n") + RX_UART,
     True, "device_class: problem"),
    ("output voltage low with diagnostics none", ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST.replace(
        "      - device_id: r1\n", "      - device_id: r1\n        output_voltage_low: {}\n") + RX_UART,
     True, "entity_category: diagnostic"),
    ("output voltage low default with basic", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART,
     True, "Output voltage low / fuse"),
    ("output voltage low bad sensor option", ULTRA + TELEMETRY + RX_LIST.replace(
        "      - device_id: r1\n", "      - device_id: r1\n        output_voltage_low:\n          accuracy_decimals: 1\n") + RX_UART,
     False, "accuracy_decimals"),
    ("ultra minimal hub", ULTRA, True, None),
    # officially supported: up to 50 boards per data pin, at 200 kHz with a complete all-Ultra inventory
    (
        "ultra 50 boards, lights on boards 1 and 50",
        ULTRA.replace("num_boards: 2", "num_boards: 50") + complete_pin(50)
        + "light:\n"
        + ULTRA_MONO.format(name="first", board=1, channel=1)
        + ULTRA_MONO.format(name="last", board=50, channel=5),
        True,
        None,
    ),
    (
        "ultra 50 boards, a light on board 51",
        ULTRA.replace("num_boards: 2", "num_boards: 50") + complete_pin(50) + "light:\n" +
        ULTRA_MONO.format(name="x", board=51, channel=1),
        False,
        "boards count from 1: board 1 .. ",
    ),
    ("ultra num_boards 65", ULTRA.replace("num_boards: 2", "num_boards: 65"), False, "64"),
    # every line: the port type's board limit, and a Dig2Analog-Plus-5ch pin's speed table (1..20 boards 800 kHz,
    # 21..34 600 kHz, 35..50 200 kHz) needs a complete all-Ultra inventory to leave 800 kHz
    ("ultra pin 20 boards: 800 kHz, nothing more", ULTRA.replace("num_boards: 2", "num_boards: 20"), True, None),
    ("ultra line speed text sensor", ULTRA + "text_sensor:\n  - platform: ultra_line\n    ultra_line_id: u\n"
     "    line_speed:\n      name: Line speed\n", True, None),
    ("ultra pin 21 boards without an inventory", ULTRA.replace("num_boards: 2", "num_boards: 21"), False,
     "a Dig2Analog-Plus-5ch pin of 21 boards runs at 600 kHz"),
    ("ultra pin 21 boards: inventory_complete named", ULTRA.replace("num_boards: 2", "num_boards: 21"), False,
     "line_rate: inventory_complete: true"),
    ("ultra pin 21 boards, complete inventory", ULTRA.replace("num_boards: 2", "num_boards: 21") + complete_pin(21),
     True, None),
    ("ultra pin 35 boards, bootloaders without 200 kHz", ULTRA.replace("num_boards: 2", "num_boards: 35") +
     complete_pin(35, mask=3), False, "bootloader_profile_mask with profile 3 (8)"),
    ("ultra pin 35 boards, a board without its descriptor", ULTRA.replace("num_boards: 2", "num_boards: 35") +
     complete_pin(35, skip=(35,)), False, "a descriptors: file for board 35 (or every_board_descriptor:"),
    # "every board on this pin: this descriptor file", stated in the YAML; a board's own entry still allowed
    ("ultra pin 50 boards, every board one descriptor", ULTRA.replace("num_boards: 2", "num_boards: 50") +
     complete_pin(50, skip=range(1, 51)).replace("  descriptors:\n", "") + f"  every_board_descriptor: {BOOT_INFO}\n",
     True, None),
    ("ultra pin 35 boards, every board one descriptor, board 4 its own", ULTRA.replace("num_boards: 2", "num_boards: 35") +
     complete_pin(35, skip=set(range(1, 36)) - {4}) + f"  every_board_descriptor: {BOOT_INFO}\n", True, None),
    ("ultra pin 35 boards, every_board_descriptor needs the rest still", ULTRA.replace("num_boards: 2", "num_boards: 35") +
     f"  every_board_descriptor: {BOOT_INFO}\n", False, "line_rate: inventory_complete: true"),
    ("ultra pin 35 boards, every_board_descriptor: no descriptor missing", ULTRA.replace("num_boards: 2", "num_boards: 35") +
     f"  every_board_descriptor: {BOOT_INFO}\n", False, "!a descriptors: file"),
    ("ultra every_board_descriptor missing file", ULTRA + "  every_board_descriptor: no/such/file.bin\n", False,
     "no/such/file.bin"),
    ("ultra every_board_descriptor on a Diff-Power port", ULTRA + "  every_board_descriptor: " + BOOT_INFO + "\n" +
     TELEMETRY + RX_UART, False, "every_board_descriptor is for a Dig2Analog-Plus-5ch data pin"),
    ("ultra pin 51 boards", ULTRA.replace("num_boards: 2", "num_boards: 51") + complete_pin(51), False,
     "a Dig2Analog-Plus-5ch data pin chains at most 50 boards"),
    ("ultra Diff-Power port 9 receivers", ULTRA.replace("num_boards: 2", "num_boards: 9") + TELEMETRY + RX_UART, False,
     "a Diff-Power port chains at most 8 boards"),
    ("ultra rgbww light with board effects", ULTRA + ULTRA_RGBWW, True, None),
    (
        "ultra monochromatic lights on board 2",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=2, channel=1) + ULTRA_MONO.format(name="b", board=2, channel="ch2"),
        True,
        None,
    ),
    (
        "ultra board out of range",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=3, channel=1),
        False,
        "board 3 out of range",
    ),
    (
        "ultra channel used twice",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + ULTRA_MONO.format(name="b", board=1, channel=1),
        False,
        "is used by both",
    ),
    (
        "ultra incomplete rgb",
        ULTRA + "light:\n  - platform: dig2analog_plus\n    name: x\n    red: 1\n    green: 2\n",
        False,
        "give the channels of one light type",
    ),
    (
        "ultra constant_brightness on rgb",
        ULTRA + "light:\n  - platform: dig2analog_plus\n    name: x\n    red: 1\n    green: 2\n    blue: 3\n"
        "    constant_brightness: true\n",
        False,
        "constant_brightness needs cold_white and warm_white",
    ),
    (
        "ultra color loop on a monochromatic light",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    effects:\n      - ultra_color_loop:\n",
        False,
        "needs an RGB light",
    ),
    (
        "ultra effect on a non-Ultra light",
        LEDC + ULTRA
        + "light:\n  - platform: monochromatic\n    name: m\n    output: o1\n    effects:\n      - ultra_pulse:\n"
        + ULTRA_MONO.format(name="a", board=1, channel=1),
        False,
        "only runs on analog lights (dig2analog_plus, ur_analog_m)",
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
        ULTRA + "sensor:\n  - platform: ultra_line\n    update_interval: 10s\n    queue_peak:\n      name: q\n"
        "    memory_peak:\n      name: m\n    frame_build_time:\n      name: b\n    rmt_underruns:\n      name: u\n"
        "    frame_rate:\n      name: r\n",
        True,
        None,
    ),
    (
        "ultra diagnostics unknown sensor",
        ULTRA + "sensor:\n  - platform: ultra_line\n    heap_free:\n      name: h\n",
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
    ("ultra min_pulse 50 ns", ULTRA + "  min_pulse: 50ns\n", True, None),
    ("ultra pulse_skip_rate 1 kHz", ULTRA + "  pulse_skip_rate: 1kHz\n", True, None),
    ("ultra pulse_skip_rate 2 kHz", ULTRA + "  pulse_skip_rate: 2000Hz\n", True, None),
    ("ultra pulse_skip_rate 500 Hz", ULTRA + "  pulse_skip_rate: 500Hz\n", False, "pulse_skip_rate must be 2kHz"),
    # YAML reads a bare on / off as a boolean: the modes that are called on / off take it
    ("ultra bare on / off (yaml booleans)", ULTRA + "  dithering: on\n  signal_loss: off\n  dim_floor: off\n", True, None),
    ("ultra bare on for signal_loss", ULTRA + "  signal_loss: on\n", False, "'on' is not an option here; use one of hold, off, fade_off"),
    ("ultra dithering bogus", ULTRA + "  dithering: maybe\n", False, "Unknown value 'maybe'"),
    ("ultra stagger adaptive", ULTRA + "  stagger: adaptive\n", True, None),
    ("ultra stagger adaptive_spread", ULTRA + "  stagger: adaptive_spread\n", True, None),
    ("ultra stagger spread", ULTRA + "  stagger: spread\n", True, None),
    ("ultra stagger fixed_spread", ULTRA + "  stagger: fixed_spread\n", True, None),
    # protocol 0.3: signal loss to the power-on scene; the trips' retry policy (CONFIG 24)
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
    # each channel's own ceiling on the line's boards: the Dig2Analog-Plus-5ch rev 0.4 R / G / B 3 A, W1 / W2 5 A
    (
        "ultra max_channel_current at each Dig2Analog-Plus-5ch channel's own ceiling",
        ULTRA + "  power_limit:\n    channel_current: 6A\n"
        "    max_channel_current: {red: 3A, green: 3A, blue: 3A, white1: 5A, white2: 5A}\n"
        "    max_board_current: 8A\n" + ULTRA_RGBWW,
        True,
        None,
    ),
    (
        "ultra max_channel_current 5A on every Dig2Analog-Plus-5ch channel",
        ULTRA + "  power_limit:\n    channel_current: 4A\n    max_channel_current: 5A\n" + ULTRA_RGBWW,
        False,
        "max_channel_current red (CH1) 5 A is above the Dig2Analog-Plus-5ch's 3 A for that channel (each channel's own "
        "ceiling: CH1 3 A, CH2 3 A, CH3 3 A, CH4 5 A, CH5 5 A",
    ),
    (
        "ultra max_channel_current W2 above 5A on a Dig2Analog-Plus-5ch",
        ULTRA + "  power_limit:\n    channel_current: 4A\n    max_channel_current: {white2: 5.5A}\n" + ULTRA_RGBWW,
        False,
        "max_channel_current white2 (CH5) 5.5 A is above the Dig2Analog-Plus-5ch's 5 A for that channel",
    ),
    (
        "ultra max_channel_current 3A uniform on a Dig2Analog-Plus-5ch (below every ceiling)",
        ULTRA + "  power_limit:\n    channel_current: 4A\n    max_channel_current: 3A\n" + ULTRA_RGBWW,
        True,
        None,
    ),
    (
        "ultra max_channel_current 5A on a Diff-Power-UR-Analog-M",
        ULTRA + "  power_limit:\n    channel_current: 4A\n    max_channel_current: 5A\nlight:\n"
        + AM_MONO.format(name="a", board=1, channel=1),
        True,
        None,
    ),
    (
        "ultra max_channel_current 6A on a Diff-Power-UR-Analog-M",
        ULTRA + "  power_limit:\n    channel_current: 4A\n    max_channel_current: {ch1: 6A}\nlight:\n"
        + AM_MONO.format(name="a", board=1, channel=1),
        False,
        "max_channel_current red (CH1) 6 A is above the Diff-Power-UR-Analog-M's 5 A",
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
        "      - lambda: 'id(u).calibrate_current(1);'\n",
        True,
        None,
    ),
    (
        "ultra calibrate_current action",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - ultra_line.calibrate_current:\n          id: u\n"
        "      - ultra_line.calibrate_current:\n          id: u\n          boards: [2]\n          level: 50%\n"
        "      - ultra_line.cancel_calibration: u\n",
        True,
        None,
    ),
    (
        "ultra show_version action",
        ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
        "      - ultra_line.show_version: u\n"
        "      - ultra_line.show_version:\n          id: u\n          board: 2\n"
        "      - ultra_line.show_version:\n          id: u\n          board: !lambda 'return 1;'\n",
        True,
        None,
    ),
    (
        "ultra show_version board out of range",
        ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
        "      - ultra_line.show_version:\n          id: u\n          board: 65\n",
        False,
        "board 65: a line has at most 64 boards",
    ),
    # boards count from 1 (board 1 = the first board after the controller), like num_boards: board 0 is refused
    # everywhere a position is given; left out, a light is on board 1 and an action takes every board
    ("board 0: refused", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=0, channel=1), False,
     "board 0: boards count from 1: the first board after the controller is board 1"),
    ("boards 1 and 2 of 2", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1)
     + ULTRA_MONO.format(name="b", board=2, channel=1), True, None),
    ("board 3 of 2: refused", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=3, channel=1), False,
     "board 3 out of range, 'u' has num_boards: 2 (boards count from 1: board 1 .. 2)"),
    ("board 65: refused", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=65, channel=1), False,
     "board 65: a line has at most 64 boards"),
    ("board left out: board 1",
     ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1).replace("    board: 1\n", "")
     + ULTRA_MONO.format(name="b", board=1, channel=1), False, "board 1 CH1 is used by both 'a' and 'b'"),
    ("ur_digital_m board 0: refused", ULTRA_DP + "light:\n" + ur_digital_m_light(board=0), False,
     "board 0: boards count from 1"),
    ("ur_digital_m board left out: board 1", ULTRA_DP + "light:\n" + ur_digital_m_light().replace("    board: 1\n", ""),
     True, None),
    ("update inventory board 0: refused", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(0), False,
     "board 0: boards count from 1"),
    ("update inventory boards 1 and 2 of 2", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(1) + inv_item(2), True,
     "!can't report its identity"),
    ("update inventory board 3 of 2: refused", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(3), False,
     "update inventory: board 3 out of range, 'u' has num_boards: 2 (boards count from 1: board 1 .. 2)"),
    ("update inventory board 2 of 2 missing: named", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(1), True,
     "board 2 of 'u' can't report its identity"),
    ("descriptors board 0: refused", ULTRA + f"  descriptors:\n    - board: 0\n      file: {BOOT_INFO}\n", False,
     "board 0: boards count from 1"),
    ("current_calibration board 0: refused", ULTRA + "  current_calibration:\n    on_boot: true\n    boards: [0]\n", False,
     "board 0: boards count from 1"),
    ("current_calibration boards 1 and 2 of 2", ULTRA + "  current_calibration:\n    on_boot: true\n    boards: [1, 2]\n",
     True, None),
    ("calibrate_current action board 0: refused",
     ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
     "      - ultra_line.calibrate_current:\n          id: u\n          boards: [0]\n", False, "board 0: boards count from 1"),
    ("show_version board 0: refused",
     ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
     "      - ultra_line.show_version:\n          id: u\n          board: 0\n", False, "board 0: boards count from 1"),
    ("show_version board -1 (the old 'every board'): refused",
     ULTRA + "button:\n  - platform: template\n    name: v\n    on_press:\n"
     "      - ultra_line.show_version:\n          id: u\n          board: -1\n", False, "board -1: boards count from 1"),
    (
        "ultra calibrate_current sequential (removed: always one board at a time)",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - ultra_line.calibrate_current:\n          id: u\n          sequential: true\n",
        False,
        "[sequential] is an invalid option",
    ),
    (
        "ultra calibrate_current level 3 %",
        ULTRA + "button:\n  - platform: template\n    name: c\n    on_press:\n"
        "      - ultra_line.calibrate_current:\n          id: u\n          level: 3%\n",
        False,
        "level must be all",
    ),
    (
        "ultra current_calibration schedule",
        ULTRA + "  current_calibration:\n    on_boot: true\n    interval: 7d\n    only_when_off: true\n"
        "    boards: [1, 2]\n",
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
        ULTRA + "  current_calibration:\n    on_boot: true\n    boards: [3]\n",
        False,
        "current_calibration: board 3 out of range",
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
        "      - ultra_line.switch_boards_to_ultra:\n          id: u\n"
        "      - ultra_line.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ultra\n"
        "      - ultra_line.switch_boards_to_ultra:\n          id: u\n          boot_protocol: !lambda 'return 0;'\n",
        True,
        None,
    ),
    (
        "ultra switch_boards_to_ultra to WS2805",
        ULTRA + "button:\n  - platform: template\n    name: s\n    on_press:\n"
        "      - ultra_line.switch_boards_to_ultra:\n          id: u\n          boot_protocol: ws2805\n",
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
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1)
        + "    restore_mode: ALWAYS_ON\n    initial_state:\n      brightness: 40%\n    power_on_scene: true\n",
        True,
        None,
    ),
    (
        "ultra power_on_scene with ALWAYS_OFF",
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    power_on_scene: true\n",
        False,
        "power_on_scene shows the state the light comes back in",
    ),
    # Protocol 0.3 stores levels, with no receiver-side effects.
    ("ultra power_on_effect removed", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_effect: Candle\n", False, "power_on_effect"),
    ("ultra power_on_level percentage", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_level: 60%\n", True, None),
    ("ultra power_on_level dark", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_level: 0%\n", True, None),
    ("ultra power_on_level no effects", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    power_on_level: 40%\n", True, None),
    ("ultra signal_loss_level", ULTRA + ULTRA_CCT + "    signal_loss_level:\n      brightness: 20%\n      color_temperature: 2700 K\n      fade: 500ms\n", True, None),
    ("ultra power_on_level rejects effect", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_level:\n      effect: Candle\n", False, "effect"),
    ("ultra signal_loss_level rejects effect", ULTRA + "light:\n" + MONO_CANDLE + "    signal_loss_level:\n      effect: Candle\n", False, "effect"),
    ("ultra power_on_level short fade", ULTRA + "light:\n" + MONO_CANDLE + "    power_on_level:\n      fade: 2s\n", False, "at most"),
    ("ultra power_on_level colour temperature", ULTRA + ULTRA_CCT + "    power_on_level:\n      color_temperature: 2700 K\n", True, None),
    ("ultra power_on_level colour temperature outside LEDs", ULTRA + ULTRA_CCT + "    power_on_level:\n      color_temperature: 2200 K\n", False, "color_temperature must be between"),
    ("ultra power_on_level with follow scene", ULTRA + "light:\n" + MONO_CANDLE + "    restore_mode: RESTORE_DEFAULT_OFF\n    power_on_scene: true\n    power_on_level: 50%\n", False, "give power_on_scene or power_on_level"),
    ("ultra power_on_level red on cct", ULTRA + ULTRA_CCT + "    power_on_level:\n      red: 50%\n", False, "red needs red, green and blue"),
    ("ultra power_on_level mixed whites", ULTRA + ULTRA_CCT + "    power_on_level:\n      color_temperature: 3000 K\n      cold_white: 50%\n", False, "give color_temperature or cold_white"),
    ("ultra power_on_level RGB and white interlock", ULTRA + ULTRA_RGBWW + "    color_interlock: true\n    power_on_level:\n      red: 100%\n      color_temperature: 3000 K\n", False, "with color_interlock give either"),
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
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    dim_to_warm: true\n",
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
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1)
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
        "'m' is not an analog light (dig2analog_plus, ur_analog_m)",
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
        ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1)
        + "    effect_glide: false\n    effects:\n      - flicker:\n      - strobe:\n      - ultra_breathe:\n",
        True,
        None,
    ),
    (
        "ultra white-only light",
        ULTRA + "light:\n  - platform: dig2analog_plus\n    name: w\n    board: 1\n    white: 4\n",
        True,
        None,
    ),
    ("ultra three lines at rmt_symbols 192", lines(3), False, "need 9 x 64"),
    ("ultra three lines at rmt_symbols 128", lines(3, 128), True, None),
    ("ultra two lines on an esp32-s3 at 192", lines(2), False, "use_dma: true", "esp32-s3-devkitc-1"),
    ("ultra two lines on an esp32-s3 at 96", lines(2, 96), True, None, "esp32-s3-devkitc-1"),
    ("effect rate 20", ULTRA + "  effect_rate: 20\n", True, None),
    ("effect rate zero", ULTRA + "  effect_rate: 0\n", False, "at least 1"),
    ("per-light effect rate", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    effect_rate: 10\n", True, None),
    ("per-light effect rate zero", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) + "    effect_rate: 0\n", False, "at least 1"),
    ("line rate policy", ULTRA + "  line_rate:\n    inventory_complete: true\n    bootloader_profile_mask: 15\n    path_distortion_ns: 218\n    window: 3s\n    minimum_frames: 50\n    error_ratio: 2%\n    bad_windows: 2\n    initial_profile: 1\n", True, None),
    ("line reserved profile", ULTRA + "  line_rate:\n    initial_profile: 4\n", False, "at most 3"),
    ("ultra update minimal", ULTRA + UPD_NET + UPD, True, None),
    (
        "ultra update all options",
        ULTRA + UPD_NET + UPD
        + "    source: quinled/some-firmware\n    channel: beta\n    public_keys: test\n"
        "    update_interval: 1h\n    channel_select:\n      name: fw channel\n"
        "    status:\n      name: fw status\n    report:\n      name: fw report\n"
        + UPD_INV + inv_item(1, "        metadata_base: 0x08002000\n        metadata_bytes: 256\n"
                                "        bootloader_bytes: 8192\n        blackout: 30ms\n")
        + inv_item(2, "        uid: 000102030405060708090a0b\n        return_channel: true\n"),
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
    # no production signing keys yet: said at validation (QuinLED provisions them); not with the test keys
    ("ultra update without production keys: warned", ULTRA + UPD_NET + UPD, True, "no production signing keys yet"),
    ("ultra update test keys: no key warning", ULTRA + UPD_NET + UPD + "    public_keys: test\n", True,
     "!no production signing keys yet"),
    # the one-image updater's options are gone: the plan picks every node's image (port job)
    ("ultra update image_target gone", ULTRA + UPD_NET + UPD + "    image_target: 0101010103000011080021000000df00\n", False,
     "image_target"),
    ("ultra update passes gone", ULTRA + UPD_NET + UPD + "    passes: 2\n", False, "passes"),
    ("ultra update auto_retry gone", ULTRA + UPD_NET + UPD + "    auto_retry: 0\n", False, "auto_retry"),
    # the inventory: boards the return channel can't read
    ("ultra update inventory", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(1) + inv_item(2), True,
     "!can't report its identity"),
    ("ultra update inventory: a shipped boot identity, no file", ULTRA + UPD_NET + UPD + UPD_INV +
     "      - board: 1\n        boot_identity: dig2analog_plus_5ch_rev04_bl4\n"
     "      - board: 2\n        boot_identity: dig2analog_plus_5ch_rev04_bl4\n", True, "!can't report its identity"),
    ("ultra update inventory: boot_identity and boot_info", ULTRA + UPD_NET + UPD + UPD_INV +
     inv_item(1, "        boot_identity: dig2analog_plus_5ch_rev04_bl4\n"), False, "exactly one"),
    # (a name without its bootloader version, as a product name would be, is no identity)
    ("ultra update inventory: a boot identity by product name only", ULTRA + UPD_NET + UPD + UPD_INV +
     "      - board: 1\n        boot_identity: dig2analog_plus_5ch\n", False, "dig2analog_plus_5ch_rev04_bl4"),
    ("ultra update inventory: neither boot_identity nor boot_info", ULTRA + UPD_NET + UPD + UPD_INV +
     "      - board: 1\n", False, "exactly one"),
    ("ultra update inventory: an unknown boot identity", ULTRA + UPD_NET + UPD + UPD_INV +
     "      - board: 1\n        boot_identity: not_shipped\n", False, "dig2analog_plus_5ch_rev04_bl4"),
    ("ultra update one-way boards unprovisioned: warned", ULTRA + UPD_NET + UPD, True, "can't report its identity"),
    ("ultra update inventory: a board twice", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(1) + inv_item(1), False,
     "each board once"),
    ("ultra update inventory: a board out of range", ULTRA + UPD_NET + UPD + UPD_INV + inv_item(3), False,
     "out of range"),
    ("ultra update inventory: part of a layout", ULTRA + UPD_NET + UPD + UPD_INV +
     inv_item(1, "        metadata_base: 0x08002000\n"), False, "all three or none"),
    ("ultra update inventory: return_channel without its uid", ULTRA + UPD_NET + UPD + UPD_INV +
     inv_item(1, "        return_channel: true\n"), False, "needs the board's uid"),
    ("ultra update inventory: blackout over a minute", ULTRA + UPD_NET + UPD + UPD_INV +
     inv_item(1, "        blackout: 2min\n"), False, "blackout"),
    # the board limit is the port type's: a Diff-Power port 8, a Dig2Analog-Plus-5ch data pin 50
    ("ultra update Diff-Power port, 8 receivers", ULTRA.replace("num_boards: 2", "num_boards: 8") + TELEMETRY + RX_UART +
     UPD_NET + UPD, True, None),
    ("ultra update Diff-Power port, 9 receivers", ULTRA.replace("num_boards: 2", "num_boards: 9") + TELEMETRY + RX_UART +
     UPD_NET + UPD, False, "a Diff-Power port chains at most 8 boards"),
    ("ultra update data pin, 50 boards", ULTRA.replace("num_boards: 2", "num_boards: 50") + complete_pin(50) + UPD_NET +
     UPD, True, None),
    ("ultra update data pin, 51 boards", ULTRA.replace("num_boards: 2", "num_boards: 51") + UPD_NET + UPD, False,
     "a Dig2Analog-Plus-5ch data pin chains at most 50 boards"),
    # the port type is the line's (the hub's port_type:), not the update entity's
    ("ultra update port_type gone", ULTRA + UPD_NET + UPD + "    port_type: d2a_pin\n", False, "port_type"),
    ("ultra line given as Diff-Power, 9 boards", ULTRA.replace("num_boards: 2", "num_boards: 9") +
     "  port_type: diff_power\n" + UPD_NET + UPD, False, "a Diff-Power port chains at most 8 boards"),
    ("ultra line given as Diff-Power, 8 boards", ULTRA.replace("num_boards: 2", "num_boards: 8") +
     "  port_type: diff_power\n", True, None),
    ("ultra line given as a data pin, 50 boards", ULTRA.replace("num_boards: 2", "num_boards: 50") +
     "  port_type: d2a_pin\n" + complete_pin(50), True, None),
    ("ultra telemetry line given as a data pin", ULTRA + "  port_type: d2a_pin\n" + TELEMETRY + RX_UART, False,
     "port_type: d2a_pin, but the line is a Diff-Power port (telemetry)"),
    ("ultra digital port given as a data pin", ULTRA_DP.replace("  digital:\n", "  port_type: d2a_pin\n  digital:\n"),
     False, "port_type: d2a_pin, but the line is a Diff-Power port (a digital port (digital:))"),
    # a Diff-Power Ultra Receiver light makes its line a Diff-Power port (8 receivers), also without telemetry
    ("ultra receiver light on a line given as a data pin", ULTRA + "  port_type: d2a_pin\n" + "light:\n" +
     ur_digital_m_light(board=2), False,
     "but the line is a Diff-Power port (a Diff-Power Ultra Receiver light 's' (ur_digital_m))"),
    ("ultra receiver light: the line is a Diff-Power port, 9 boards", ULTRA.replace("num_boards: 2", "num_boards: 9") +
     "light:\n" + ur_digital_m_light(board=9), False, "a Diff-Power port chains at most 8 boards"),
    ("ultra receiver light: the line is a Diff-Power port, 8 boards", ULTRA.replace("num_boards: 2", "num_boards: 8") +
     "light:\n" + ur_digital_m_light(board=8), True, None),
    # the light platform says what the board is: ur_analog_m a receiver (a Diff-Power port), dig2analog_plus a
    # Dig2Analog-Plus-5ch (no return channel: never one of the receivers)
    ("ultra Analog-M light: the line is a Diff-Power port, 9 boards", ULTRA.replace("num_boards: 2", "num_boards: 9") +
     "light:\n" + AM_MONO.format(name="a", board=9, channel=1), False, "a Diff-Power port chains at most 8 boards"),
    ("ultra one board, two light platforms", ULTRA + "light:\n" + ULTRA_MONO.format(name="a", board=1, channel=1) +
     AM_MONO.format(name="b", board=1, channel=2), False, "the lights of one board use its platform"),
    ("ultra telemetry: a Dig2Analog-Plus-5ch among the receivers", ULTRA + TELEMETRY + RX_LIST + RX_UART + "light:\n" +
     ULTRA_MONO.format(name="a", board=2, channel=1), False, "passes no answers on, so it goes behind the receivers"),
    ("ultra telemetry: a Dig2Analog-Plus-5ch behind the receivers", ULTRA + TELEMETRY + "    receivers: 1\n" + RX_UART +
     "light:\n" + AM_MONO.format(name="a", board=1, channel=1) + ULTRA_MONO.format(name="b", board=2, channel=1),
     True, None),
    ("ultra stream light on a digital port given as a data pin",
     ULTRA_DP.replace("  digital:\n", "  port_type: d2a_pin\n  digital:\n") + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT,
     False, "a Diff-Power Ultra Receiver light 'stream' (ur_digital_m_stream)"),
    ("ultra line port_type bogus", ULTRA + "  port_type: rs485\n", False, "Unknown value"),
    ("ultra update on a digital port", ULTRA_DP + UPD_NET + UPD, True, None),
    ("ultra update channel bogus", ULTRA + UPD_NET + UPD + "    channel: nightly\n", False, "Unknown value"),
    ("ultra update two per line", ULTRA + UPD_NET + UPD + UPD.replace("id: fw", "id: fw2").replace("name: fw", "name: fw2").replace("update:\n", ""), False, "one per line"),
    # the line is quiet for 2.1 s after a port update: signal_loss off / fade_off must outlast it
    (
        "ultra update with signal_loss fade_off 1s",
        ULTRA + "  signal_loss: fade_off\n  signal_loss_timeout: 1s\n" + UPD_NET + UPD,
        False,
        "must be at least 2.1s",
    ),
    (
        "ultra update with signal_loss off 2s",
        ULTRA + "  signal_loss: \"off\"\n  signal_loss_timeout: 2s\n" + UPD_NET + UPD,
        False,
        "signal_loss_timeout (2000 ms) must be at least 2.1s",
    ),
    (
        "ultra update with signal_loss off 2.1s",
        ULTRA + "  signal_loss: \"off\"\n  signal_loss_timeout: 2100ms\n" + UPD_NET + UPD,
        True,
        None,
    ),
    (
        "ultra update with signal_loss hold 500ms",
        ULTRA + "  signal_loss_timeout: 500ms\n" + UPD_NET + UPD,
        True,
        None,
    ),
    # a receiver that doesn't answer has its own Recover firmware button (a line with a port update entity)
    ("ultra update: receivers get Recover firmware", ULTRA + TELEMETRY + RX_UART + UPD_NET + UPD, True,
     "name: Port R2 recover firmware"),
    ("ultra update: Recover firmware on the receiver's device",
     ULTRA + TELEMETRY + RX_LIST + RX_UART + UPD_NET + UPD,
     True, "name: Recover firmware"),
    ("ultra telemetry without an update entity: no Recover firmware", ULTRA + TELEMETRY + RX_UART, True,
     "!recover firmware"),
    ("ultra update without http_request", ULTRA + "wifi:\n  ssid: x\n  password: xxxxxxxx\n" + UPD, False, "http_request"),
    (
        "ultra update actions",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - update.perform:\n          id: fw\n"
        "      - ultra_line.abort_update: fw\n",
        True,
        None,
    ),
    (
        "ultra update recover gone",
        ULTRA + UPD_NET + UPD
        + "button:\n  - platform: template\n    name: b\n    on_press:\n"
        "      - ultra_line.recover:\n          id: fw\n",
        False,
        "ultra_line.recover",
    ),
    ("ultra esp32-c2 rejected (no RMT)", ULTRA, False, "RMT", "esp32-c2-devkitm-1"),
    # Diff-Power receiver telemetry
    ("ultra telemetry minimal (receivers: every board, on the hub's device)", ULTRA + TELEMETRY + RX_UART, True, None),
    # Diff-Power ports
    ("diffpower port minimal", DP_PORT.replace("    detect_pin: GPIO27\n", ""), True, None),
    # its settings entities: Turn-off period and Wake timeout (each can be left out), the controller's dark offsets
    ("diffpower turn-off period entity", DP_PORT, True, "name: Port 1 turn-off period"),
    ("diffpower wake timeout entity", DP_PORT, True, "name: Port 1 wake timeout"),
    # the wake timeline (diffpower_core.h WakeLog): its length, the line, the warning
    ("diffpower wake duration entity", DP_PORT, True, "name: Port 1 wake duration"),
    ("diffpower wake timeline entity", DP_PORT, True, "name: Port 1 wake timeline"),
    ("diffpower wake warning entity", DP_PORT, True, "name: Port 1 wake warning"),
    ("diffpower wake timeline left out",
     DP_PORT + "    wake_duration: false\n    wake_timeline: false\n    wake_warning: false\n", True, "!wake duration"),
    ("diffpower settings entities left out",
     DP_PORT + "    standby_after_number: false\n    wake_timeout_number: false\n    settings: false\n", True,
     "!turn-off period"),
    (
        "diffpower port: a plain light with its own turn-off period, options",
        DP_PORT + "    standby_after: 5min\n    hotplug_delay: 2s\n    boot_margin: 300ms\n    notice_time: 250ms\n"
        "    min_off_time: 2s\n    min_on_time: 5min\n    power_mode: always_on\n"
        "    lights:\n      - id: mono\n        standby_after: 30min\n    standby_cycles: false\n"
        "    standby_hours:\n      name: Port 1 standby\n    detect_drops: false\n" + DP_MONO,
        True,
        None,
    ),
    ("diffpower during_dark without a source", DP_PORT + "    power_mode: during_dark\n", False,
     "needs the sunrise / sunset: from Home Assistant (api:"),
    ("diffpower during_dark from Home Assistant (api:, its time)", DP_PORT + "    power_mode: during_dark\n" + DP_HA,
     True, "sun_entity: sun.sun"),
    ("diffpower during_dark, api: without a time",
     DP_PORT + "    power_mode: during_dark\n" + DP_HA.split("time:")[0], False, "needs the time now"),
    ("diffpower during_dark, api: and two times",
     DP_PORT + "    power_mode: during_dark\n" + DP_HA + "  - platform: sntp\n    id: t2\n", False,
     "say which one with time_id"),
    ("diffpower during_dark, api: and two times, time_id",
     DP_PORT + "    power_mode: during_dark\n    time_id: t2\n" + DP_HA + "  - platform: sntp\n    id: t2\n", True,
     None),
    ("diffpower during_dark from Home Assistant, the sun the backup",
     DP_PORT + "    power_mode: during_dark\n    sun_id: s\n" + DP_SUN + "api:\n", True, None),
    ("diffpower sun_entity", DP_PORT + "    power_mode: during_dark\n    sun_entity: sun.home\n" + DP_HA, True,
     "sun_entity: sun.home"),
    ("diffpower sun_entity not an entity id", DP_PORT + "    sun_entity: sun\n" + DP_HA, False, "sun_entity"),
    ("diffpower two ports, two sun entities",
     DP_PORT + "    sun_entity: sun.home\n  - id: p2\n    name: Port 2\n    enable_pin: GPIO25\n" + DP_HA, False,
     "sun_entity: one for all ports"),
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
    # the power mode Off (round 12): quoted, or bare (YAML's false)
    ("diffpower power mode off", DP_PORT + '    power_mode: "off"\n', True, None),
    ("diffpower power mode off, bare", DP_PORT + "    power_mode: off\n", True, None),
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
        DP_PORT + ULTRA + "  port: p1\n" + "light:\n  - platform: ur_analog_m\n    id: l1\n    name: l1\n"
        "    board: 1\n    channel: 1\n" + DP_WAKE.format(light="l1"),
        True,
        None,
    ),
    (
        "diffpower.wake: a light of an Ultra line without a port",
        DP_PORT + ULTRA + "light:\n  - platform: dig2analog_plus\n    id: l1\n    name: l1\n"
        "    board: 1\n    channel: 1\n" + DP_WAKE.format(light="l1"),
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
        "    detect_drops: false\n    power_mode_select: false\n    wake: false\n    standby_after_number: false\n"
        "    wake_timeout_number: false\n    wake_duration: false\n    wake_timeline: false\n    wake_warning: false\n"
        "  - id: p2\n    name: Port 1\n    enable_pin: GPIO32\n    detect_pin: GPIO33\n    status: false\n"
        "    problem: false\n    standby_cycles: false\n    standby_hours: false\n    detect_drops: false\n"
        "    power_mode_select: false\n    wake: false\n    standby_after_number: false\n    wake_timeout_number: false\n"
        "    wake_duration: false\n    wake_timeline: false\n    wake_warning: false\n",
        True,
        None,
    ),
    (
        "ultra line on a port, a light with its own turn-off period",
        DP_PORT + ULTRA + "  port: p1\n" + "light:\n  - platform: ur_analog_m\n    name: l1\n    board: 1\n"
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
        DP_PORT + "ultra_line:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    port: p1\n"
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
        + "light:\n  - platform: ur_analog_m\n    name: l1\n    board: 2\n    channel: 1\n    device_id: shelf_1\n"
        "  - platform: ur_analog_m\n    name: l2\n    board: 2\n    channel: 2\n",
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
        "ultra telemetry lifetime / diag intervals, its entities renamed",
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
        "ultra_line:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
        "  - id: b\n    pin: GPIO17\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
        + RX_UART.replace("GPIO17", "GPIO18"),
        False,
        "Duplicate",
    ),
    (
        "ultra telemetry two ports with names",
        "ultra_line:\n  - id: a\n    pin: GPIO16\n    num_boards: 1\n    telemetry:\n      uart_id: rx\n"
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
        ULTRA + "sensor:\n  - platform: ultra_line\n    queue_peak:\n      name: q\n",
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
    # Home Assistant: the update rate, the live twins, the Port text,
    # the Identify button, the controller's totals, DE and /RE tied on a shared UART
    ("ultra telemetry update_rate normal", ULTRA + TELEMETRY + "    update_rate: normal\n" + RX_UART, True, None),
    # the RAM estimate (a codegen warning above ~60 KB on a classic ESP32): 3 Diff-Power ports x 6 receivers (a light
    # each) with full diagnostics warn; the same with basic diagnostics, or one small line, don't
    ("ultra RAM warning: 18 receivers, full diagnostics (~65 KB)", ram_lines(3, 6), True, "of the ESP32's RAM"),
    ("ultra RAM: 18 receivers, basic diagnostics with model texts", ram_lines(3, 6, "basic"), True, "of the ESP32's RAM"),
    ("ultra RAM: one small line", ram_lines(1, 2), True, "!of the ESP32's RAM"),
    ("ultra telemetry update_rate bogus", ULTRA + TELEMETRY + "    update_rate: ludicrous\n" + RX_UART, False,
     "Unknown value"),
    # the line's health (round 12): with diagnostics full (disabled until enabled), not with basic unless listed; the
    # reply rate adaptive by default, adaptive_rate: false pins it
    ("ultra telemetry health entities (full)", ULTRA + TELEMETRY + RX_UART, True, "name: Port reply rate"),
    ("ultra telemetry health: missing answers", ULTRA + TELEMETRY + RX_UART, True, "name: Port missing answers"),
    ("ultra telemetry no health entities with basic", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_UART, True,
     "!reply rate"),
    ("ultra telemetry a health entity listed with basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n    port_reply_rate:\n      name: Cable rate\n" + RX_UART, True,
     "name: Cable rate"),
    ("ultra telemetry adaptive_rate false", ULTRA + TELEMETRY + "    adaptive_rate: false\n" + RX_UART, True, None),
    ("ultra telemetry adaptive_rate bogus", ULTRA + TELEMETRY + "    adaptive_rate: sometimes\n" + RX_UART, False,
     "adaptive_rate"),
    # the commissioning (round 12): its buttons and texts by default, none with commissioning: false; the actions
    # (a line, or every line without an id) need a line with it
    ("ultra commissioning entities", ULTRA + TELEMETRY + RX_UART, True, "name: Port finalize"),
    ("ultra commissioning report text", ULTRA + TELEMETRY + RX_UART, True, "name: Port commissioning report"),
    ("ultra commissioning off", ULTRA + TELEMETRY + "    commissioning: false\n" + RX_UART, True, "!finalize"),
    ("ultra commission_min_voltage 40 %", ULTRA + TELEMETRY + "    commission_min_voltage: 40%\n" + RX_UART, False,
     "commission_min_voltage"),
    ("ultra commission actions",
     ULTRA + TELEMETRY + RX_UART + "button:\n  - platform: template\n    name: Commission all\n    on_press:\n"
     "      - ultra_line.commission:\n      - ultra_line.finalize: u\n", True, None),
    ("ultra commission action without commissioning",
     ULTRA + TELEMETRY + "    commissioning: false\n" + RX_UART + "button:\n  - platform: template\n    name: C\n"
     "    on_press:\n      - ultra_line.commission: u\n", False, "has no telemetry with commissioning"),
    ("ultra telemetry live twins on devices", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Live power"),
    ("ultra telemetry Port text on a receiver's device", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Port\n"),
    ("ultra telemetry Identify button", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Identify"),
    ("ultra telemetry no Port text without a device", ULTRA + TELEMETRY + RX_UART, True, "!R1 port"),
    ("ultra telemetry port live power and energy", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Port 1 energy"),
    # the current calibration: the line's button and each receiver's, enabled; the receiver's Calibration, Current
    # model and Last calibrated (basic and full), Connected load (full, disabled until enabled)
    ("ultra telemetry line calibrate button (enabled)", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Port 1 calibrate current\n        disabled_by_default: false"),
    ("ultra telemetry line version button (disabled)", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Port 1 show firmware version\n        disabled_by_default: true"),
    ("ultra telemetry receiver calibrate button", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Calibrate current\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry receiver Last calibrated", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Last calibrated\n            device_id: r2\n            disabled_by_default: false"),
    ("ultra telemetry receiver Current model (basic)",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "name: Current model"),
    ("ultra telemetry receiver Connected load (full, disabled)", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Connected load\n            device_id: r1\n            disabled_by_default: true"),
    ("ultra telemetry no Connected load with basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "!Connected load"),
    ("power model full scale default disabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: CH1 full-scale input current\n            device_id: r1\n            disabled_by_default: true"),
    ("power model JSON full enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Power model\n            device_id: r1\n            disabled_by_default: false"),
    ("power model JSON basic enabled", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True,
     "name: Power model\n            device_id: r1\n            disabled_by_default: false"),
    ("digital calibration counts basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "name: Detected pixel count"),
    ("digital calibration configured count basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "name: Configured pixel count"),
    ("digital calibration notice basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "name: Pixel count"),
    ("digital calibration cancel basic",
     ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True, "name: Cancel calibration"),
    ("power model JSON none omitted", ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST + RX_UART, True,
     "!name: Power model"),
    ("power model explicit output enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART + "light:\n"
     + AM_MONO.format(name="a", board=1, channel=1) + "    output_model: {}\n", True,
     "name: a output model\n      disabled_by_default: false"),
    ("power model idle default disabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Idle input current\n            device_id: r1\n            disabled_by_default: true"),
    ("power model basic omits currents", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART,
     True, "!full-scale input current"),
    ("power model explicit sensor enable", ULTRA + TELEMETRY + "    diagnostics: none\n    receivers:\n"
     "      - device_id: r1\n        ch5_full_scale_input_current:\n          disabled_by_default: false\n"
     + RX_UART, True, "name: CH5 full-scale input current"),
    ("power model light output schema", ULTRA + TELEMETRY + RX_LIST + RX_UART + "light:\n"
     + AM_MONO.format(name="a", board=1, channel=1)
     + "    output_model:\n      name: Strip output model\n      disabled_by_default: false\n", True,
     "name: Strip output model"),
    # the LED supply: a shown diagnostic (enabled, with basic too), next to the bus voltage
    ("ultra telemetry LED supply enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: LED supply\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry LED supply with basic", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True,
     "name: LED supply"),
    # the cable-drop advisor: a text and two numbers per receiver, shown with basic too, not with none; the
    # port's 36 V rules: a text and their own problem, always
    ("ultra telemetry Cable drop enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Cable drop\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry Full-load voltage enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Full-load voltage\n            device_id: r2\n            disabled_by_default: false"),
    ("ultra telemetry Full-load drop enabled", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Full-load drop\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry cable drop with basic", ULTRA + TELEMETRY + "    diagnostics: basic\n" + RX_LIST + RX_UART, True,
     "name: Full-load drop"),
    ("ultra telemetry no cable drop with none", ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST + RX_UART, True,
     "!name: Full-load voltage"),
    ("ultra telemetry cable drop explicit with none",
     ULTRA + TELEMETRY + "    diagnostics: none\n    receivers:\n      - device_id: r1\n        cable_drop:\n"
     "          name: Cable drop\n" + RX_UART, True, "name: Cable drop"),
    ("ultra telemetry port load rules", ULTRA + TELEMETRY + RX_LIST + RX_UART, True, "name: Port 1 load rules"),
    ("ultra telemetry port load rules problem", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Port 1 load rules problem"),
    ("ultra telemetry port load rules with none", ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST + RX_UART, True,
     "name: Port 1 load rules problem"),
    # supply dips: a problem below dip_warning (80 %); the Supply note (enabled); the commissioning's 80 %
    ("ultra telemetry Supply note", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Supply note\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry dip_warning default", ULTRA + TELEMETRY + RX_UART, True, "dip_warning: 0.8"),
    ("ultra telemetry dip_warning 85%", ULTRA + TELEMETRY + "    dip_warning: 85%\n" + RX_UART, True, "dip_warning: 0.85"),
    ("ultra telemetry dip_warning 95% (above the receivers' own 90 %)",
     ULTRA + TELEMETRY + "    dip_warning: 95%\n" + RX_UART, False, "dip_warning"),
    ("ultra commission_min_voltage default 80 %", ULTRA + TELEMETRY + RX_UART, True, "commission_min_voltage: 0.8"),
    # the controller's settings (made at codegen: their options here), the receivers' Settings text, Position and
    # UID shown (the event log's replacements)
    ("ultra telemetry settings false", ULTRA + TELEMETRY + "    settings: false\n" + RX_UART, True, "settings: false"),
    ("ultra telemetry supply_rating", ULTRA + TELEMETRY + "    supply_rating: 480W\n" + RX_UART, True,
     "supply_rating: 480.0"),
    ("ultra telemetry supply_rating bogus", ULTRA + TELEMETRY + "    supply_rating: -5W\n" + RX_UART, False,
     "supply_rating"),
    ("ultra telemetry receiver Settings text", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Settings\n            device_id: r1\n            disabled_by_default: false"),
    ("ultra telemetry Position and UID shown", ULTRA + TELEMETRY + RX_LIST + RX_UART, True,
     "name: Position and UID\n            device_id: r2\n            disabled_by_default: false"),
    ("ultra telemetry no calibration texts with none",
     ULTRA + TELEMETRY + "    diagnostics: none\n" + RX_LIST + RX_UART, True, "!Last calibrated"),
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
     parlio(lanes=1, telemetry=False).replace("ultra_line:\n",
                                              "  - id: more\n    pins: [GPIO26]\nultra_line:\n"),
     False, "the ESP32-P4 has one PARLIO TX unit", P4),
    ("parlio 4 lanes sharing a UART without receiver enable pins", parlio(lanes=4, uarts=1, re=False), False,
     "use the same uart 'g0'", P4),
    ("parlio on an esp32", parlio(lanes=2, telemetry=False), False, "ultra_parlio (PARLIO TX, 16 lines)"),
    ("parlio on an esp32-s3", parlio(lanes=2, telemetry=False), False, "ultra_parlio",
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
     False, "lane 0: bit0_high becomes 400 ns at 5 MHz", P4),
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
    ("parlio 16 lanes, buffer_time 400 us", parlio(telemetry=False, extra_bus="    buffer_time: 400us\n"), False,
     "at least 553 us", P4),
    # lanes don't use the RMT: 16 lanes and the P4's 4 RMT lines (3 x 48 + DMA) fit; the RMT lines still add up
    ("parlio 16 lanes and 4 RMT lines (3 x 48 symbols + DMA)",
     parlio(telemetry=False) + PARLIO_RMT_LINES + "  - id: m3\n    pin: GPIO39\n    num_boards: 1\n    use_dma: true\n",
     True, None, P4),
    ("parlio 16 lanes and 5 RMT lines at 48 symbols",
     parlio(telemetry=False) + PARLIO_RMT_LINES + "  - id: m3\n    pin: GPIO39\n    num_boards: 1\n    rmt_symbols: 48\n"
     "  - id: m4\n    pin: GPIO40\n    num_boards: 1\n    rmt_symbols: 48\n", False, "need 5 x 48", P4),
    # ---- ur_digital_m
    ("ur_digital_m minimal", ULTRA_DP + UR_DIGITAL_M, True, None),
    ("ur_digital_m board v203", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    board_type: v203\n"), True, "!Daylight Arc"),
    ("ur_digital_m board v407", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    board_type: v407\n"), True, "Heartbeat"),
    ("ur_digital_m v407 palette", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    board_type: v407\n    palette: Iridescent\n"), True, None),
    ("ur_digital_m v203 cannot use v407 palette", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    palette: Iridescent\n"), False, "unknown palette"),
    ("ur_digital_m board invalid", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    board_type: other\n"), False, "Unknown value"),
    ("ur_digital_m board segments disagree", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    board_type: v203\n")
     + ur_digital_m_light(name="other", segment=1, extra="    board_type: v407\n"), False, "board_type differs"),
    ("ur_digital_m sk6812_rgbw WRGB", ULTRA_DP + "light:\n" + ur_digital_m_light(chip="sk6812_rgbw", extra="    color_order: WRGB\n"), True, None),
    ("ur_digital_m ws2814 default RGBW", ULTRA_DP + "light:\n" + ur_digital_m_light(chip="ws2814"), True, None),
    ("ur_digital_m ws2805 five channels",
     ULTRA_DP + "light:\n" + ur_digital_m_light(chip="ws2805", extra="    color_order: RGBCW\n    cold_white_share: 40%\n"), True, None),
    ("ur_digital_m ws2812b color_order with a W", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    color_order: RGBW\n"), False, "3 letters"),
    ("ur_digital_m ws2814 color_order RGB", ULTRA_DP + "light:\n" + ur_digital_m_light(chip="ws2814", extra="    color_order: RGB\n"), False, "4 letters"),
    ("ur_digital_m color_order twice the same channel", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    color_order: GGB\n"), False, "each of RGB once"),
    ("ur_digital_m cold_white_share on a plain chip", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    cold_white_share: 40%\n"), False, "WS2805"),
    ("ur_digital_m unknown chip", ULTRA_DP + "light:\n" + ur_digital_m_light(chip="apa102"), False, "apa102"),
    ("ur_digital_m segment beyond the strip", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    segment_start: 70\n"), False, "beyond num_leds"),
    ("ur_digital_m segment range beyond the strip",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    segment_start: 40\n    segment_length: 40\n"), False, "beyond num_leds"),
    ("ur_digital_m board out of range", ULTRA_DP + "light:\n" + ur_digital_m_light(board=2), False, "boards count from 1: board 1 .. "),
    ("ur_digital_m 4 segments of one receiver",
     ULTRA_DP + "light:\n" + "".join(
         ur_digital_m_light(name=f"s{k}", segment=k, extra=f"    segment_start: {k * 15}\n    segment_length: 15\n") for k in range(4)),
     True, None),
    ("ur_digital_m the same segment twice", ULTRA_DP + "light:\n" + ur_digital_m_light(name="a") + ur_digital_m_light(name="b"), False, "is used by"),
    ("ur_digital_m segments disagreeing on the chip",
     ULTRA_DP + "light:\n" + ur_digital_m_light(name="a") + ur_digital_m_light(name="b", segment=1, chip="sk6812"), False, "differs"),
    ("ur_digital_m receiver settings on one light of the receiver",
     ULTRA_DP + "light:\n" + ur_digital_m_light(name="a", extra="    max_fps: 60\n") + ur_digital_m_light(name="b", segment=1), True, None),
    ("ur_digital_m receiver settings agreeing",
     ULTRA_DP + "light:\n" + ur_digital_m_light(name="a", extra="    max_fps: 60\n") + ur_digital_m_light(name="b", segment=1, extra="    max_fps: 60\n"),
     True, None),
    ("ur_digital_m receiver settings disagreeing",
     ULTRA_DP + "light:\n" + ur_digital_m_light(name="a", extra="    max_fps: 60\n") + ur_digital_m_light(name="b", segment=1, extra="    max_fps: 30\n"),
     False, "say it on one light"),
    ("ur_digital_m segment 4", ULTRA_DP + "light:\n" + ur_digital_m_light(segment=4), False, "segment"),
    ("ur_digital_m refresh_interval over a second is fine (the beacon)", ULTRA_DP + "  refresh_interval: 5s\n" + UR_DIGITAL_M, True, None),
    ("ur_digital_m next to an analog light (dig2analog_plus, ur_analog_m)",
     ULTRA_DP + "light:\n" + ULTRA_MONO.format(name="m", board=1, channel=1) + ur_digital_m_light(name="u"), False, "is a digital port"),
    ("ur_digital_m exclusive with a streaming light",
     ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM + ur_digital_m_light(extra="    exclusive_with: streamer\n"), True, None),
    ("ur_digital_m exclusive with an ur_digital_m_stream light",
     ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT + ur_digital_m_light(extra="    exclusive_with: streamer\n"), True, None),
    ("ur_digital_m exclusive with itself", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    id: me\n    exclusive_with: me\n"), False, "itself"),
    ("ur_digital_m exclusive with something that is no light",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    exclusive_with: u\n"), False, "exclusive_with"),
    ("ur_digital_m a custom effect id", ULTRA_DP + "light:\n" + ur_digital_m_light(extra=ur_digital_m_effects(("Mine", 130))), True, None),
    ("ur_digital_m an effect name twice", ULTRA_DP + "light:\n" + ur_digital_m_light(extra=ur_digital_m_effects(("Solid", 130))), False, "twice"),
    ("ur_digital_m only the listed effects", ULTRA_DP + "light:\n" + ur_digital_m_light(extra=ur_digital_m_effects(("Mine", 130), only=True)), True, None),
    ("ur_digital_m fx 0", ULTRA_DP + "light:\n" + ur_digital_m_light(extra=ur_digital_m_effects(("x", 0))), False, "fx"),
    ("ur_digital_m an ultra_line effect",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    effects:\n      - ultra_pulse:\n"), False, "ultra_pulse"),
    ("ur_digital_m an ESPHome effect", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    effects:\n      - random:\n"), True, None),
    ("ur_digital_m the palette by name", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    palette: ocean\n"), True, None),
    ("ur_digital_m an unknown palette", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    palette: nope\n"), False, "unknown palette"),
    ("ur_digital_m colours as hex and lists",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    color_2: 0xFF8000\n    color_3: [0, 64, 255]\n"), True, None),
    ("ur_digital_m color_2 quoted hex", ULTRA_DP + UR_DIGITAL_M + "    color_2: '#aB12ef'\n", True, None),
    ("ur_digital_m color_2 list", ULTRA_DP + UR_DIGITAL_M + "    color_2: [12, 34, 255]\n", True, None),
    ("ur_digital_m a bad colour", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    color_2: [1, 2]\n"), False, "red, green, blue"),
    ("ur_digital_m current and limits",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    led_current: 18mA\n    current_limit: 2A\n    max_fps: 60\n"), True, None),
    ("ur_digital_m standalone and store",
     ULTRA_DP + "light:\n" + ur_digital_m_light(
         extra="    store_after: 30s\n    standalone:\n      power_on: true\n      on_signal_loss: true\n      signal_loss_timeout: 5s\n"),
     True, None),
    ("ur_digital_m store_after under 5 s", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    store_after: 1s\n"), False, "5"),
    ("ur_digital_m too many pixels", ULTRA_DP + "light:\n" + ur_digital_m_light(leds=3000), False, "600"),
    ("ur_digital_m the action",
     ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    id: strip\n") + UR_DIGITAL_M_ACTION.format(
         id="strip", fields="          color_2: 0xFF0000\n          speed: 200\n          palette: Ocean\n"), True, None),
    ("ur_digital_m the action on something that is no light", ULTRA_DP + UR_DIGITAL_M + UR_DIGITAL_M_ACTION.format(id="u", fields=""), False, "u"),
    ("ur_digital_m on a normal line, board 1 of 2 (any position)", ULTRA + UR_DIGITAL_M, True, None),
    # ---- chain mode: the receiver at any position of a normal Ultra line, mixed with other boards, CONFIG 75 `mode:`
    ("chain: the receiver last of 3, mode chain",
     ULTRA_CH + "light:\n" + ULTRA_MONO.format(name="m0", board=1, channel=1) + ur_digital_m_light(board=3, extra="    mode: chain\n"), True, None),
    ("chain: mode auto, the default",
     ULTRA_CH + "light:\n" + ULTRA_MONO.format(name="m0", board=1, channel=1) + ur_digital_m_light(board=3), True, None),
    ("chain: mode auto written", ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, extra="    mode: auto\n"), True, None),
    ("chain: the receiver alone on a normal line", ULTRA.replace("num_boards: 2", "num_boards: 1") + UR_DIGITAL_M, True, None),
    ("chain: Dig2Analog-Plus-5ch lights in front (boards 1 and 2)",
     ULTRA_CH + "light:\n" + ULTRA_MONO.format(name="m0", board=1, channel=1) + ULTRA_MONO.format(name="m1", board=2, channel=2) +
     ur_digital_m_light(board=3, extra="    mode: chain\n"), True, None),
    ("chain: 4 segments of the last board",
     ULTRA_CH + "light:\n" + "".join(
         ur_digital_m_light(name=f"s{k}", board=3, segment=k, extra=f"    segment_start: {k * 15}\n    segment_length: 15\n") for k in range(4)),
     True, None),
    ("chain: the mode on one segment, the others silent",
     ULTRA_CH + "light:\n" + ur_digital_m_light(name="a", board=3, extra="    mode: chain\n") + ur_digital_m_light(name="b", board=3, segment=1), True, None),
    ("chain: the mode agreeing on two segments",
     ULTRA_CH + "light:\n" + ur_digital_m_light(name="a", board=3, extra="    mode: chain\n") +
     ur_digital_m_light(name="b", board=3, segment=1, extra="    mode: chain\n"), True, None),
    ("chain: the mode disagreeing on two segments",
     ULTRA_CH + "light:\n" + ur_digital_m_light(name="a", board=3, extra="    mode: chain\n") +
     ur_digital_m_light(name="b", board=3, segment=1, extra="    mode: auto\n"), False, "say it on one light"),
    ("chain: standalone power-on and store",
     ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, extra="    store_after: 30s\n    standalone:\n      power_on: true\n      on_signal_loss: true\n"),
     True, None),
    ("chain: with telemetry on the line (3 receivers)",
     ULTRA_CH + TELEMETRY + "    name: Port 1\n    receivers:\n      - device_id: r1\n      - device_id: r2\n      - device_id: shelf_1\n" +
     RX_UART + "light:\n" + ur_digital_m_light(board=3, extra="    device_id: shelf_1\n    mode: chain\n"), True, None),
    ("chain: telemetry covers the first receivers only (the receiver is not polled)",
     ULTRA_CH + TELEMETRY + "    name: Port 1\n    receivers:\n      - device_id: r1\n      - device_id: r2\n" + RX_UART + "light:\n" +
     ur_digital_m_light(board=3, extra="    mode: chain\n"), True, None),
    ("chain: in the middle (board 2 of 3)", ULTRA_CH + "light:\n" + ur_digital_m_light(board=2), True, None),
    ("chain: the first board of 3", ULTRA_CH + "light:\n" + ur_digital_m_light(board=1, extra="    mode: chain\n"), True, None),
    ("chain: two receivers on one line",
     ULTRA_CH + "light:\n" + ur_digital_m_light(name="a", board=2, extra="    mode: chain\n") + ur_digital_m_light(name="b", board=3, extra="    mode: chain\n"),
     True, None),
    ("chain: a mixed Diff-Power line of 8 boards (receivers at 1, 4, 8; analog boards between)",
     ULTRA.replace("num_boards: 2", "num_boards: 8") + "  port_type: diff_power\n" + "light:\n" +
     ur_digital_m_light(name="a", board=1, extra="    mode: chain\n") + ULTRA_MONO.format(name="m1", board=2, channel=1) +
     ULTRA_MONO.format(name="m2", board=3, channel=1) + ur_digital_m_light(name="b", board=4) +
     "".join(ULTRA_MONO.format(name=f"m{k}", board=k, channel=1) for k in (5, 6, 7)) + ur_digital_m_light(name="c", board=8),
     True, None),
    ("chain: a mixed Diff-Power line of 9 boards", ULTRA.replace("num_boards: 2", "num_boards: 9") + "  port_type: diff_power\n" +
     "light:\n" + ur_digital_m_light(name="a", board=1) + ur_digital_m_light(name="b", board=9), False,
     "a Diff-Power port chains at most 8 boards"),
    ("smoothing: dither", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    pixel_smoothing: dither\n"), True, None),
    ("smoothing: off", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    pixel_smoothing: off\n"), True, None),
    ("smoothing: not a mode", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    pixel_smoothing: fast\n"), False, "pixel_smoothing"),
    ("smoothing: two lights of a receiver disagree",
     ULTRA_DP + "light:\n" + ur_digital_m_light(name="a", extra="    pixel_smoothing: round\n") +
     ur_digital_m_light(name="b", segment=1, extra="    pixel_smoothing: dither\n"), False, "differs from"),
    ("chain: board out of range", ULTRA_CH + "light:\n" + ur_digital_m_light(board=4), False, "boards count from 1: board 1 .. "),
    ("chain: next to an analog light (dig2analog_plus, ur_analog_m) on its own board",
     ULTRA_CH + "light:\n" + ULTRA_MONO.format(name="m", board=3, channel=1) + ur_digital_m_light(board=3), False, "is an analog light's (dig2analog_plus, ur_analog_m)"),
    ("chain: mode stream needs a digital port", ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, extra="    mode: stream\n"), False,
     "mode: stream needs a digital port"),
    ("chain: mode chain on a digital port", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    mode: chain\n"), False,
     "mode: chain is for a normal Ultra line"),
    ("chain: exclusive_with has no stream light to share with",
     ULTRA_CH + "light:\n" + UR_DIGITAL_M_STREAM + ur_digital_m_light(board=3, extra="    exclusive_with: streamer\n"), False,
     "exclusive_with is for a digital port"),
    ("chain: stream_wins has no meaning",
     ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, extra="    standalone:\n      stream_wins: true\n"), False, "stream_wins"),
    ("chain: an unknown mode", ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, extra="    mode: bogus\n"), False, "bogus"),
    ("chain: a digital port, mode stream", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    mode: stream\n"), True, None),
    ("chain: a digital port, mode auto", ULTRA_DP + "light:\n" + ur_digital_m_light(extra="    mode: auto\n"), True, None),
    ("chain: a stream light needs a digital port", ULTRA_CH + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT, False, "not a digital port"),
    # ---- the digital port itself
    ("digital port minimal", ULTRA_DP, True, None),
    ("digital port with a reset time", ULTRA_DP.replace("  digital:\n", "  digital:\n    reset_time: 80us\n"), True, None),
    ("digital port reset time too short", ULTRA_DP.replace("  digital:\n", "  digital:\n    reset_time: 10us\n"), False, "50us"),
    ("digital port with two boards", ULTRA_DP.replace("num_boards: 1", "num_boards: 2"), False, "one receiver"),
    ("digital port with telemetry needs the direction pin",
     ULTRA_DP + TELEMETRY + RX_LIST + RX_UART, False, "direction_pin"),
    ("digital port with telemetry",
     ULTRA_DP + TELEMETRY + RX_LIST.replace("      - device_id: r2\n", "") + "    direction_pin: GPIO4\n" + RX_UART, True, None),
    ("digital port with a firmware update entity (a Diff-Power port of one receiver)",
     ULTRA_DP + UPD_NET + UPD, True, "!digital port"),
    ("digital port with an analog light (dig2analog_plus, ur_analog_m)",
     ULTRA_DP + "light:\n" + ULTRA_MONO.format(name="m", board=1, channel=1), False, "digital port"),
    ("digital port: the retry action",
     ULTRA_DP + "button:\n  - platform: template\n    name: R\n    on_press:\n      - ultra_line.retry_receiver:\n          id: u\n",
     True, None),
    # ---- ur_digital_m_stream
    ("ur_digital_m removed 400 kHz chip", ULTRA_DP + "light:\n" + ur_digital_m_light(chip="ws2811_400k"), False, "Unknown value"),
    ("chain: removed 400 kHz chip", ULTRA_CH + "light:\n" + ur_digital_m_light(board=3, chip="ws2811_400k"), False, "Unknown value"),
    ("stream light removed 400 kHz chipset", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT.replace("chipset: ws2812", "chipset: ws2811_400k"),
     False, "Unknown value"),
    ("stream light minimal", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT, True, None),
    ("stream light sk6812 rgbw", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT.replace("chipset: ws2812", "chipset: sk6812\n    channel_colors: GRBW"), True, None),
    ("stream light custom timing",
     ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT.replace("    chipset: ws2812\n", "    bit0_high: 350ns\n    bit0_low: 900ns\n    bit1_high: 700ns\n"
                                          "    bit1_low: 550ns\n    reset_time: 80us\n"), True, None),
    ("stream light with a chipset and timing", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT + "    bit0_high: 350ns\n    bit0_low: 900ns\n"
     "    bit1_high: 700ns\n    bit1_low: 550ns\n", False, "chipset"),
    ("stream light without a chipset or timing", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT.replace("    chipset: ws2812\n", ""), False, "chipset"),
    ("stream light on a line that is no digital port", ULTRA + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT, False, "not a digital port"),
    ("stream light twice on a port", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT + UR_DIGITAL_M_STREAM_LIGHT.replace("id: streamer", "id: s2").replace("name: stream", "name: s2"),
     False, "one strip"),
    ("stream light too many pixels", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT.replace("num_leds: 30", "num_leds: 3000"), False, "2500"),
    ("stream light with effects", ULTRA_DP + "light:\n" + UR_DIGITAL_M_STREAM_LIGHT + "    effects:\n      - pulse:\n      - addressable_rainbow:\n", True, None),
]


# per receiver, by board_type: V203 600 RGB / 500 RGBW / 400 RGBCCT, V407 800 / 600 / 500 (the firmware's descriptors)
for chip, bpp in (("ws2812b", 3), ("ws2812b_classic", 3), ("sk6812", 3), ("sk6812_rgbw", 4), ("ws2814", 4), ("ws2805", 5),
                  ("ws2815", 3)):
    for board, limits, counts in (("v203", {3: 600, 4: 500, 5: 400}, (-1, 0, 1)), ("v407", {3: 800, 4: 600, 5: 500}, (0, 1))):
        limit = limits[bpp]
        for count in (limit + k for k in counts):
            CASES.append((f"pixel cap {board} {chip} {count}",
                          ULTRA_DP + "light:\n" + ur_digital_m_light(chip=chip, leds=count, extra=f"    board_type: {board}\n"),
                          count <= limit, None if count <= limit else f"at most {limit} per receiver (board_type {board})"))


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


def check_update_hardware() -> bool:
    """Firmware/jig identities are opaque, exact 16-byte tuples; no product defaults."""
    sys.path.insert(0, str(ROOT / "components"))
    from ultra_line import update as upd
    import esphome.config_validation as cv
    assert upd._hex_bytes("0101010103000011080021000000df00",16) == "0101010103000011080021000000df00"
    for value in ("", "01", "x"*32, "0"*34):
        try:
            upd._hex_bytes(value,16)
            return False
        except cv.Invalid:
            pass
    print("ok   update immutable tuple validation (no board-id/revision defaults)")
    # the shipped boot identities: each key names one exact bootloader (<board>_rev<RR>_bl<N>, what its export says),
    # each export is valid, and no two keys give one export or one bootloader of one board
    blobs, exact = set(), set()
    for key, blob in upd.BOOT_IDENTITIES.items():
        suffix = f"_rev{blob[12]:02d}_bl{blob[4]}"
        board = key[:-len(suffix)] if key.endswith(suffix) else None
        if not board or key != key.lower() or upd._check_boot_info(blob) != blob:
            return False
        if blob in blobs or (board, blob[12], blob[4]) in exact:
            return False
        blobs.add(blob)
        exact.add((board, blob[12], blob[4]))
    try:   # (an entry whose key says another bootloader than its export carries is refused when the table is built)
        upd._boot_identity("dig2analog_plus_5ch_rev04_bl3", upd.BOOT_IDENTITIES["dig2analog_plus_5ch_rev04_bl4"].hex())
        return False
    except ValueError:
        pass
    print(f"ok   shipped boot identities: {len(upd.BOOT_IDENTITIES)}, each key = its board, revision and bootloader version")
    return True


def check_group_budget() -> bool:
    """The codegen's group budget (__init__.py _max_group_commands): the commands one Home Assistant change to every
    light can give the busiest board, which the hub's start lead covers."""
    sys.path.insert(0, str(ROOT / "components"))
    from ultra_line import _max_group_commands as budget

    plain = [{"board": 1} for _ in range(5)]
    effects = [{"board": 1, "effects": [{"ultra_breathe": {}}]} for _ in range(5)]
    strobes = [{"board": 1, "effects": [{"ultra_strobe": {"colors": [{}] * 8}}], "power_on_scene": True}] * 5
    cases = [
        ("no lights", [], 1),
        ("5 plain lights: a SET each", plain, 5),
        ("5 host effect lights: a SET each", effects, 5),
        ("5 strobes with 8 steps and power_on_scene", strobes, 10),
        ("the busiest board counts", plain + [{"board": 2, "effects": [{"ultra_breathe": {}}]}], 5),
    ]
    ok = True
    for name, lights, want in cases:
        got = budget(lights)
        miss = "" if got == want else f" (want {want})"
        print(f"{'ok  ' if got == want else 'FAIL'} group budget, {name}: {got}{miss}")
        ok &= got == want
    return ok


def entity_codegen(path):
    """Generate in memory: real ESPHome validation/codegen, no ESP-IDF toolchain needed."""
    from esphome.core import CORE
    from esphome.config import read_config
    from esphome.__main__ import generate_cpp_contents

    CORE.config_path = Path(path)
    config = read_config({})
    if config is None:
        return 1
    CORE.config = config
    generate_cpp_contents(config)
    sys.stdout.reconfigure(encoding="utf-8")
    print(CORE.cpp_global_section + CORE.cpp_main_section)
    return 0


def check_rate_request_entities() -> bool:
    """Verify receiver binding, diagnostic category and the default visibility in real codegen."""
    cases = [
        ("full", "", RX_LIST, 2, True),
        ("basic", "    diagnostics: basic\n", RX_LIST, 0, True),
        ("explicit", "    diagnostics: none\n", RX_LIST.replace(
            "      - device_id: r1\n", "      - device_id: r1\n        rate_request:\n"
            "          disabled_by_default: false\n"), 1, False),
    ]
    ok = True
    WORK.mkdir(parents=True, exist_ok=True)
    for name, diagnostics, receivers, count, disabled in cases:
        path = WORK / f"rate-request-{name}.yaml"
        path.write_text(BASE.format(board="esp32dev", components=(ROOT / "components").as_posix()) +
                        ULTRA + TELEMETRY + diagnostics + receivers + RX_UART, encoding="utf-8", newline="\n")
        proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--entity-codegen", str(path)],
                              capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        registrations = [line for line in proc.stdout.splitlines()
                         if 'App.register_binary_sensor(' in line and '"Lower line rate requested"' in line]
        good = proc.returncode == 0 and len(registrations) == count
        good &= proc.stdout.count('->set_rate_request(') == count
        good &= all('category:diagnostic' in line and ('disabled_by_default' in line) == disabled
                    for line in registrations)
        print(f"{'ok  ' if good else 'FAIL'} rate request entity codegen: {name}")
        if not good:
            print(textwrap.indent(proc.stderr[-1500:], "     | "))
        (WORK / f"rate-request-{name}.cpp").write_text(proc.stdout, encoding="utf-8", newline="\n")
        ok &= good
    return ok


def cpp_enum(text, cls, enum):
    """{name: slot} of `enum` inside `class cls` of a header (slots count from 0 in order)"""
    start = text.index(f"class {cls}")
    m = re.compile(rf"enum {enum} : uint8_t \{{(.*?)\}};", re.S).search(text, start)
    names = [re.sub(r"\s*=.*", "", n).strip() for n in m.group(1).split(",") if n.strip()]
    return {n: i for i, n in enumerate(names)}


def bound_slots(code, register, name, setter):
    """The slots `setter` binds the registered entity `name` to (in generated C++)"""
    slots = []
    for var in re.findall(rf'App\.register_{register}\((\w+), "{re.escape(name)}"', code):
        slots += [int(i) for i in re.findall(rf"->{setter}\((\d+), {var}\)", code)]
    return slots


def check_cable_drop_entities() -> bool:
    """The cable-drop advisor's receiver entities and the port's 36 V rules entities in real codegen: each bound to
    the slot the C++ enum names (ultra_receiver.h), the port's problem to its own setter, the receivers' ones only
    with diagnostics basic / full, the port's always."""
    header = (ROOT / "components/ultra_line/ultra_receiver.h").read_text(encoding="utf-8")
    rx_value = cpp_enum(header, "UltraReceiver", "Value")
    rx_text = cpp_enum(header, "UltraReceiver", "Text")
    port_text = cpp_enum(header, "UltraTelemetryPort", "Text")
    ok = True
    WORK.mkdir(parents=True, exist_ok=True)
    for name, diagnostics, receivers in (("full", "", 2), ("basic", "    diagnostics: basic\n", 2),
                                         ("none", "    diagnostics: none\n", 0)):
        path = WORK / f"cable-drop-{name}.yaml"
        path.write_text(BASE.format(board="esp32dev", components=(ROOT / "components").as_posix()) +
                        ULTRA + TELEMETRY + diagnostics + RX_LIST + RX_UART, encoding="utf-8", newline="\n")
        proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--entity-codegen", str(path)],
                              capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        code = proc.stdout
        good = proc.returncode == 0
        good &= bound_slots(code, "sensor", "Full-load voltage", "set_value") == [rx_value["FULL_LOAD_VOLTAGE"]] * receivers
        good &= bound_slots(code, "sensor", "Full-load drop", "set_value") == [rx_value["FULL_LOAD_DROP"]] * receivers
        good &= bound_slots(code, "text_sensor", "Cable drop", "set_text") == [rx_text["CABLE_DROP"]] * receivers
        good &= bound_slots(code, "text_sensor", "Port 1 load rules", "set_text") == [port_text["LOAD_RULES"]]
        rules = re.findall(r'App\.register_binary_sensor\((\w+), "Port 1 load rules problem"', code)
        good &= len(rules) == 1 and code.count(f"->set_load_rules_problem({rules[0]})") == 1 if rules else False
        print(f"{'ok  ' if good else 'FAIL'} cable drop / load rules entity codegen: {name}")
        if not good:
            print(textwrap.indent(proc.stderr[-1500:], "     | "))
        (WORK / f"cable-drop-{name}.cpp").write_text(proc.stdout, encoding="utf-8", newline="\n")
        ok &= good
    return ok


def check_digital_entities() -> bool:
    """These entities are created by light.py's codegen, after schema validation."""
    labels = {"Speed", "Intensity", "Palette", "Colour 2", "Colour 3", "Custom 1", "Custom 2"}
    cases = [
        ("defaults", "", labels, False),
        ("custom_controls false", "    custom_controls: false\n", labels - {"Custom 1", "Custom 2"}, False),
        ("controls false", "    controls: false\n", {"Custom 1", "Custom 2"}, False),
        ("all controls false", "    controls: false\n    custom_controls: false\n", set(), False),
        ("light device", "    device_id: r1\n", labels, True),
        ("shared receiver device", "    device_id: r1\n    controls: false\n    custom_controls: false\n"
         + ur_digital_m_light(name="edge", segment=1, extra="    device_id: r1\n    controls: false\n    custom_controls: false\n"), set(), True),
    ]
    ok = True
    WORK.mkdir(parents=True, exist_ok=True)
    for index, (name, extra, wanted, device) in enumerate(cases):
        path = WORK / f"entities{index}.yaml"
        path.write_text(BASE.format(board="esp32dev", components=(ROOT / "components").as_posix()) +
                        ULTRA_DP + UR_DIGITAL_M + "    id: strip\n" + extra, encoding="utf-8", newline="\n")
        proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--entity-codegen", str(path)],
                              capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        code = proc.stdout
        # Ignore YAML comments; inspect actual entity registration and text traits, not defaults in comments.
        code = "\n".join(line for line in code.splitlines() if not line.lstrip().startswith("//"))
        names = set(re.findall(r'App\.register_(?:number|select|text)\([^,]+, "([^"]+)"', code))
        expected = {label if device else f"strip {label}" for label in wanted}
        good = proc.returncode == 0 and names == expected
        capability_name = "Effect capabilities" if device else "strip Effect capabilities"
        registration = next((line for line in code.splitlines() if 'App.register_text_sensor(' in line
                             and f'"{capability_name}"' in line), "")
        good &= "category:diagnostic" in registration and "disabled_by_default" not in registration
        good &= "->set_capabilities_sensor(" in code and "->set_build_capabilities(" in code
        if name == "shared receiver device":
            good &= code.count("->set_capabilities_sensor(") == 1
        if "Colour 2" in wanted:
            for suffix in ("colour_2", "colour_3"):
                var = f"strip_{suffix}"
                good &= all(fragment in code for fragment in (
                    f"{var}->traits.set_min_length(7)", f"{var}->traits.set_max_length(7)",
                    f'{var}->traits.set_pattern("^#[0-9A-Fa-f]{{6}}$")',
                    f"{var}->traits.set_mode(text::TEXT_MODE_TEXT)",
                ))
                if device:
                    good &= f"{var}->set_device_(r1)" in code
                registration = next((line for line in code.splitlines() if f"App.register_text({var}," in line), "")
                good &= "category:config, icon:mdi:palette-outline" in registration
            good &= '"mdi:palette-outline"' in code
        print(f"{'ok  ' if good else 'FAIL'} ur_digital_m entities: {name}")
        if not good:
            print(f"     wanted {sorted(expected)}, got {sorted(names)}; rc={proc.returncode}")
            print(textwrap.indent(proc.stderr[-1500:], "     | "))
        (WORK / f"entities{index}.cpp").write_text(proc.stdout, encoding="utf-8", newline="\n")
        ok &= good
    print(f"ur_digital_m entity codegen: {'PASS (6 cases)' if ok else 'FAIL'}")
    return ok


def check_pin_speeds() -> bool:
    """The component's copy of a Dig2Analog-Plus-5ch pin's speed table and the board limit per port type against the
    vendored sender headers' (the limits in ultra_protocol.h, the speed edges in ultra_sender_update.h)"""
    sys.path.insert(0, str(ROOT / "components"))
    from ultra_line import D2A_PIN_SPEEDS, PORT_MAX_BOARDS
    comp = ROOT / "components" / "ultra_line"
    header = "".join((comp / name).read_text(encoding="utf-8") for name in ("ultra_protocol.h", "ultra_sender_update.h"))
    value = {k: int(v) for k, v in re.findall(r"#define (ULTRA_\w+) (\d+)u", header)}
    want = ((value["ULTRA_D2A_800_MAX_BOARDS"], 0), (value["ULTRA_D2A_600_MAX_BOARDS"], 1),
            (value["ULTRA_PORT_D2A_PIN_MAX_BOARDS"], 3))
    ok = D2A_PIN_SPEEDS == want and PORT_MAX_BOARDS == {"diff_power": value["ULTRA_PORT_DIFF_POWER_MAX_BOARDS"],
                                                       "d2a_pin": value["ULTRA_PORT_D2A_PIN_MAX_BOARDS"]}
    print(("ok   " if ok else "FAIL ") + f"pin speed table and board limits match the sender headers ({want})")
    return ok


def main() -> int:
    filters = [f.lower() for f in sys.argv[1:]]
    cases = list(enumerate(CASES))
    if filters:
        cases = [(i, c) for i, c in cases if any(f in c[0].lower() for f in filters)]
    sys.path.insert(0, str(ROOT / "components"))
    import esphome.components
    esphome.components.__path__.insert(0, str(ROOT / "components"))
    from ur_digital_m.light import _receiver_config
    conf = {"chip": "sk6812_rgbw", "color_order": ("GRBW", 0), "num_leds": 320}
    assert dict(_receiver_config(conf, []))[67] == 60
    for fps in (0, 60, 120, 1000):
        assert dict(_receiver_config(dict(conf, max_fps=fps), []))[67] == fps
        assert dict(_receiver_config(conf, [{"max_fps": fps}]))[67] == fps
    print("ok   nominal 60 fps default, explicit 0/higher, shared receiver setting")
    budget_ok = filters != [] or (check_group_budget() & check_update_hardware() & check_pin_speeds())
    entities_ok = check_digital_entities() if not filters or any("ur_digital_m" in f for f in filters) else True
    if not filters or any("cable drop" in f for f in filters):
        entities_ok &= check_cable_drop_entities()
    if not filters or any("rate request" in f for f in filters):
        entities_ok &= check_rate_request_entities()
    results = [run_case(i, *case) for i, case in cases]
    failed = results.count(False)
    print(f"\n{len(results) - failed}/{len(results)} schema cases passed")
    return 1 if failed or not budget_ok or not entities_ok else 0


if __name__ == "__main__":
    sys.exit(entity_codegen(sys.argv[2]) if sys.argv[1:2] == ["--entity-codegen"] else main())
