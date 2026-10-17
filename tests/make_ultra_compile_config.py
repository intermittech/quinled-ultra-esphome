"""Writes tests/build/ultra/ultra-example.yaml for a compile test: the Ultra example with the local components and
a few test-only additions (boot protocol, power-on scene and effects, dim to warm, the switch_boards_to_ultra button,
calibration, the version display, the board firmware update with a power supply on the line), so one compile covers
the hub's code. Also tests/build/ultra/ultra-strip.yaml: the dig2analog_ultra_strip example with a partition, effect_glide modes
and the downlight notification pattern; tests/build/ultra/ultra-telemetry.yaml: ultra-example with Diff-Power receiver
telemetry (the UART, the direction pin, every receiver sensor and text); tests/build/ultra/ultra-p4-telemetry.yaml (4
RMT lines with telemetry on a P4), ultra-p4-parlio.yaml (16 PARLIO lanes, 4 shared UARTs),
ultra-p4-parlio-ports.yaml (the same, each lane on a Diff-Power port), ultra-p4-parlio-lights.yaml (8 lanes at 10
MHz, no telemetry), ultra-digital.yaml (two digital ports without telemetry: four local-effect segments and a stream light on one,
a WS2805 strip on the other) and ultra-chain.yaml (a normal Ultra line with telemetry whose last board is an Ultra-Digital
receiver in chain mode, next to a digital port with one: both transports in one build). The examples themselves:
tests/local_examples.py.

Usage: .venv/Scripts/python tests/make_ultra_compile_config.py
then (PowerShell, ESP-IDF refuses Git Bash):
  & .\\.venv\\Scripts\\esphome.exe compile tests\\build\\ultra\\ultra-example.yaml *> tests\\build\\ultra\\compile.log
"""

from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / "build" / "ultra"

# the wiki's Home Assistant example (Ultra-Board-Settings.md, "camera mode, PWM frequency and stagger")
HA_GLOBALS = """globals:
  - id: pwm_hz                              # the PWM frequency select's value (camera mode goes back to it)
    type: float
    initial_value: "25000.0f"
"""

HA_SWITCH = """  - platform: template
    id: camera_mode
    name: Camera mode
    icon: mdi:camera
    entity_category: config
    optimistic: true
    restore_mode: RESTORE_DEFAULT_OFF       # after a reboot of the ESP the last choice is sent again
    turn_on_action:
      - lambda: |-
          id(ultra).set_pwm_frequency(-1, 50000.0f);   // preset 50 kHz: banding <= 0.24 % at 1/120 s
          id(ultra).set_dithering(-1, 1);              // off at low levels ...
          id(ultra).set_dither_threshold(-1, 0.03f);   // ... below 3 %: no dither ripple there
    turn_off_action:
      - lambda: |-
          id(ultra).set_pwm_frequency(-1, id(pwm_hz)); // back to the PWM frequency select
          id(ultra).set_dithering(-1, 0);              // and the values in dig2analog_ultra:
          id(ultra).set_dither_threshold(-1, 0.01f);
"""

HA_SELECT = """  - platform: template
    name: PWM frequency
    icon: mdi:pulse
    entity_category: config
    options: ["2.197 kHz", "3.107 kHz", "4.395 kHz", "6.215 kHz", "8.789 kHz", "12.43 kHz", "17.58 kHz", "20 kHz",
              "25 kHz", "30 kHz", "35.16 kHz", "40 kHz", "50 kHz", "70.31 kHz", "100 kHz"]   # the board presets
    initial_option: "25 kHz"                # as pwm_frequency in dig2analog_ultra:
    optimistic: true
    restore_value: true
    on_value:                               # also runs at boot with the restored option
      - lambda: |-
          id(pwm_hz) = atof(x.c_str()) * 1000.0f;      // "12.43 kHz" -> 12430 Hz: that board preset
          if (!id(camera_mode).state)
            id(ultra).set_pwm_frequency(-1, id(pwm_hz));
  - platform: template
    name: Stagger
    icon: mdi:sine-wave
    entity_category: config
    options: ["off", "fixed", "fixed_spread", "adaptive", "adaptive_spread"]   # index = stagger mode 0-4
    initial_option: "adaptive_spread"
    optimistic: true
    restore_value: true
    on_value:                               # also runs at boot with the restored option
      - lambda: id(ultra).set_stagger(-1, i);
"""

EXTRA = """
# ---- compile-test additions (not part of the example)
""" + HA_GLOBALS + """
switch:
""" + HA_SWITCH + """  - platform: template
    name: Dim to warm
    optimistic: true
    restore_mode: RESTORE_DEFAULT_ON
    turn_on_action:
      - lambda: id(rgbcct_out).set_dim_to_warm(true);
    turn_off_action:
      - lambda: id(rgbcct_out).set_dim_to_warm(false);

# board firmware update (update.py), with a supply on the line (power hold / power cycle)
http_request:

power_supply:
  - id: led_psu
    pin: GPIO13
    enable_time: 0ms
    keep_on_time: 10s

update:
  - platform: dig2analog_ultra
    id: board_fw
    name: Board firmware
    dig2analog_ultra_id: ultra
    channel: beta
    public_keys: test
    channel_select:
      name: Board firmware channel
    status:
      name: Board firmware status

# the line's diagnostics (sensor.py)
sensor:
  - platform: dig2analog_ultra
    dig2analog_ultra_id: ultra
    update_interval: 30s
    queue_peak:
      name: Ultra queue peak
    memory_peak:
      name: Ultra memory peak
    frame_build_time:
      name: Ultra frame build time
    rmt_underruns:
      name: Ultra RMT underruns
    frame_rate:
      name: Ultra frame rate

button:
  - platform: template
    name: Calibrate board 1 at 25 %
    on_press:
      - dig2analog_ultra.calibrate_current:
          id: ultra
          boards: [1]
          level: 25%
      - lambda: |-
          if (id(ultra).calibration_running())
            ESP_LOGI("test", "board %u, %u of %u", id(ultra).calibration_board(),
                     (unsigned) id(ultra).calibration_step(), (unsigned) id(ultra).calibration_count());
          id(ultra).calibrate_current(0);
  - platform: template
    name: Cancel calibration
    on_press:
      - dig2analog_ultra.cancel_calibration: ultra
  # the firmware version as blinks (protocol 0.2.1, IDENTIFY 5)
  - platform: template
    name: Show board firmware versions
    on_press:
      - dig2analog_ultra.show_version: ultra
      - dig2analog_ultra.show_version:
          id: ultra
          board: 1
      - lambda: id(ultra).show_version(0);
  # the wiki's example (Ultra-Board-Settings.md, "Boards forced to WS2805 / WS2915")
  - platform: template
    name: Switch boards to Ultra
    icon: mdi:swap-horizontal
    entity_category: config
    on_press:
      - dig2analog_ultra.switch_boards_to_ultra:
          id: ultra
          boot_protocol: auto   # what the boards store afterwards: auto (the default) or ultra
  # board firmware update (update.py): install again, abort, recover
  - platform: template
    name: Board firmware install again
    on_press:
      - update.perform:
          id: board_fw
          force_update: true
  - platform: template
    name: Board firmware abort
    on_press:
      - dig2analog_ultra.abort_update: board_fw
  - platform: template
    name: Board firmware recover
    on_press:
      - dig2analog_ultra.recover:
          id: board_fw
          lead_in: 45s
"""


STRIP_EXTRA = """
  # ---- compile-test additions: a strip across both lines, effect_glide modes, a brightness-only notification strip
  - platform: dig2analog_ultra_strip
    name: Everything
    lights: [front_1, front_2, front_3, side_rgbcct, spot_b, spot_a, side_rgb]
    effect_glide: true
    default_transition_length: 2s
    effects:
      - addressable_fireworks:
      - addressable_flicker:
  - platform: dig2analog_ultra_strip
    id: spots_notify
    name: Spots notification
    internal: true
    lights: [spot_a, spot_b]
    default_transition_length: 400ms
    effects:
      - addressable_lambda:
          name: Chase
          update_interval: 150ms
          lambda: |-
            static int pos = 0;
            if (initial_run)
              pos = 0;
            it.all() = Color::BLACK;
            it[pos % it.size()] = Color::WHITE;
            pos++;

script:
  - id: notify_chase
    mode: restart
    parameters:
      runs: int
    then:
      - light.turn_on:
          id: spots_notify
          brightness: 100%
          effect: Chase
      - delay: !lambda "return runs * 300;"
      - light.turn_off: spots_notify

button:
  - platform: template
    name: Test notification
    on_press:
      - script.execute:
          id: notify_chase
          runs: 3
"""


def strip_config() -> str:
    """tests/build/ultra/ultra-strip.yaml: the strip + partition example, a strip across both lines, effect_glide modes,
    ESPHome effects gliding on a single light, and the downlight notification pattern."""
    src = (ROOT / "examples" / "quinled-dig2analog-ultra-strip-partition.yaml").read_text(encoding="utf-8")
    src = src.replace(
        "  - source: github://intermittech/quinled-ultra-esphome@main\n",
        "  - source:\n      type: local\n      path: ../../../components\n",
    )
    assert "path: ../../../components" in src
    # host-side effects on a single light (they glide on the board), a strobe that stays hard, a board effect
    old = "    id: spot_a\n    name: Path spot A\n    board: 0\n    channel: 4\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    effect_glide: auto\n    effects:\n      - flicker:\n      - strobe:\n"
                      "      - ultra_breathe:\n")
    old = "    lights: [side_rgb, spot_a, spot_b, side_rgbcct]\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    effect_glide: false\n")
    old = "      - addressable_rainbow:\n          width: 3\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "      - strobe:\n      - flicker:\n")
    # the strips before the partition, the rest at the end
    head, part = src.split("\n  # both lines as one strip", 1)
    return head + STRIP_EXTRA.split("\nscript:\n", 1)[0] + "\n  # both lines as one strip" + part.rstrip() + "\n" \
        + "\nscript:\n" + STRIP_EXTRA.split("\nscript:\n", 1)[1]


TELEMETRY_DEVICES = """  areas:
    - id: living_room
      name: Living room
    - id: hallway
      name: Hallway
  devices:                                # each receiver a device; each light of a multi-light receiver too
    - id: p1_r1
      name: Port 1 · Receiver 1
      area_id: living_room
    - id: p1_r2
      name: Port 1 · Receiver 2
    - id: cabinet
      name: Cabinet
      area_id: living_room
    - id: shelf_1
      name: Shelf 1
      area_id: living_room
    - id: shelf_2
      name: Shelf 2
      area_id: living_room
    - id: shelf_3
      name: Shelf 3
      area_id: hallway
"""

TELEMETRY_HUB = """  trip_retries: 5                       # (0.2.5) the retry policy, stored on the boards
  trip_retry_interval: 2s
  telemetry:
    uart_id: rs485_rx
    name: Port 1
    direction_pin: GPIO4
    diagnostics: full
    supply_voltage: 48V                   # the sender (else learned)
    port_limit: 2.25A
    load_problem: 95%
    receivers:
      - device_id: p1_r1                  # RGBCCT: one light, on this device
      - device_id: p1_r2                  # four lights, each on its own device
        power:
          name: Power in
    poll_intervals:
      status: 2s
      lifetime: 10min
"""

TELEMETRY_EXTRA = """
uart:
  - id: rs485_rx
    rx_pin: GPIO17
    baud_rate: 1000000
    rx_buffer_size: 2048
"""


DIFFPOWER_CONFIG = """\
# tests/build/ultra/ultra-diffpower.yaml: a 4-port QuinLED Diff-Power sender on a classic ESP32 (diffpower): two
# Ultra lines on ports (one with telemetry), two plain strips on ports, During dark (Home Assistant's sun times, the
# sun the backup on port 1), a motion pre-wake.
esphome:
  name: d2a-diffpower

esp32:
  board: esp32dev
  framework:
    type: esp-idf

external_components:
  - source:
      type: local
      path: ../../../components
    components: [dig2analog_ultra, diffpower]

logger:
api:
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

time:
  - platform: homeassistant
    id: ha_time

sun:
  id: sun_
  latitude: 52.1
  longitude: 5.1

uart:
  - id: rs485_rx
    rx_pin: GPIO34
    baud_rate: 1000000
    rx_buffer_size: 1024

diffpower:
  - id: port1
    name: Port 1
    enable_pin: GPIO25
    detect_pin: GPIO35
    power_mode: during_dark
    sun_id: sun_
    dark_before: 45min
    dark_after: 30min
  - id: port2
    name: Port 2
    enable_pin: GPIO26
    detect_pin: GPIO36
    standby_after: 30min
  - id: port3
    name: Port 3
    enable_pin: GPIO27
    detect_pin: GPIO39
    lights: [hall_strip]
  - id: port4
    name: Port 4
    enable_pin: GPIO14
    detect_pin: GPIO32
    power_mode: always_on
    lights:
      - id: garden_strip
        standby_after: 1h

dig2analog_ultra:
  - id: line1
    pin: GPIO16
    num_boards: 2
    rmt_symbols: 128
    port: port1
    telemetry:
      uart_id: rs485_rx
      name: Port 1
      direction_pin: GPIO4
  - id: line2
    pin: GPIO17
    num_boards: 1
    rmt_symbols: 128
    port: port2

light:
  - platform: dig2analog_ultra
    dig2analog_ultra_id: line1
    name: Living cove
    board: 0
    red: 1
    green: 2
    blue: 3
    cold_white: 4
    warm_white: 5
  - platform: dig2analog_ultra
    dig2analog_ultra_id: line1
    name: Shelf 1
    board: 1
    channel: 1
    standby_after: 1h
  - platform: dig2analog_ultra
    dig2analog_ultra_id: line1
    name: Shelf 2
    board: 1
    channel: 2
  - platform: dig2analog_ultra
    dig2analog_ultra_id: line2
    name: Kitchen
    board: 0
    cold_white: 1
    warm_white: 2
  - platform: esp32_rmt_led_strip
    id: hall_strip
    name: Hall strip
    pin: GPIO18
    num_leds: 30
    chipset: ws2812
    channel_colors: GRB
    rmt_symbols: 64
  - platform: partition              # (its pixels are the hall strip's: the port powers it too)
    id: hall_door
    name: Hall door
    segments:
      - id: hall_strip
        from: 0
        to: 9
  - platform: partition              # (a partition of that partition: still the hall strip's pixels)
    id: hall_door_top
    name: Hall door top
    segments:
      - id: hall_door
        from: 0
        to: 4
  - platform: esp32_rmt_led_strip
    id: garden_strip
    name: Garden strip
    pin: GPIO19
    num_leds: 60
    chipset: ws2812
    channel_colors: GRB
    rmt_symbols: 64

binary_sensor:
  - platform: gpio
    pin: GPIO33
    name: Hall motion
    on_press:
      - diffpower.wake:
          lights: [hall_strip]
          timeout: 2min
"""


def p4_telemetry_config() -> str:
    """tests/build/ultra/ultra-p4-telemetry.yaml: a larger sender on an ESP32-P4: 4 data lines, each with telemetry on
    a UART of its own (2 receivers each, devices per receiver), within the P4's RMT (3 lines of 48 symbols and one with
    DMA: 4 transmit channels) and its UARTs (5; the log on USB Serial/JTAG)."""
    devices, uarts, hubs, lights, ports = "", "", "", "", ""
    for k in range(1, 5):
        devices += (f"    - id: p{k}_r1\n      name: Port {k} receiver 1\n"
                    f"    - id: p{k}_r2\n      name: Port {k} receiver 2\n")
        uarts += (f"  - id: rs485_rx{k}\n    rx_pin: GPIO{25 + k}\n    baud_rate: 1000000\n"
                  f"    rx_buffer_size: 1024\n")
        rmt = "    use_dma: true\n" if k == 4 else "    rmt_symbols: 48\n"
        ports += (f"  - id: dp{k}\n    name: Port {k}\n    enable_pin: GPIO{43 + k}\n"
                  f"    detect_pin: GPIO{47 + k}\n")
        hubs += (f"  - id: port{k}\n    pin: GPIO{19 + k}\n    num_boards: 2\n{rmt}    port: dp{k}\n    telemetry:\n"
                 f"      uart_id: rs485_rx{k}\n      name: Port {k}\n      direction_pin: GPIO{29 + k}\n"
                 f"      receivers:\n        - device_id: p{k}_r1\n        - device_id: p{k}_r2\n")
        lights += (f"  - platform: dig2analog_ultra\n    dig2analog_ultra_id: port{k}\n    name: Port {k} RGBCCT\n"
                   f"    device_id: p{k}_r1\n    board: 0\n    red: 1\n    green: 2\n    blue: 3\n"
                   f"    cold_white: 4\n    warm_white: 5\n")
        for ch in (1, 2):
            lights += (f"  - platform: dig2analog_ultra\n    dig2analog_ultra_id: port{k}\n"
                       f"    name: Port {k} shelf {ch}\n    board: 1\n    channel: {ch}\n")
    return ("esphome:\n  name: d2a-ultra-p4\n  devices:\n" + devices +
            "\nesp32:\n  board: esp32-p4_r3\n  cpu_frequency: 360MHz\n  framework:\n    type: esp-idf\n\n"
            "external_components:\n  - source:\n      type: local\n      path: ../../../components\n"
            "    components: [dig2analog_ultra, diffpower]\n\nlogger:\n\nuart:\n" + uarts +
            "\ndiffpower:\n" + ports +
            "\ndig2analog_ultra:\n" + hubs + "\nlight:\n" + lights)


def p4_parlio_config(ports: bool = False) -> str:
    """tests/build/ultra/ultra-p4-parlio.yaml: a 16-port sender on an ESP32-P4: 16 lanes of one PARLIO bus, telemetry
    on 4 UARTs shared by 4 lanes each (a direction pin per lane on the P4, its receiver enable on an MCP23017: any
    output pin will do), a light per lane, and an RMT line next to them. ports (ultra-p4-parlio-ports.yaml): each lane
    on its Diff-Power port (diffpower), the enables on a second MCP23017, the detects on a third."""
    de = [20, 21, 22, 23, 26, 27, 28, 29, 30, 31, 32, 33, 39, 40, 41, 42]
    uarts, hubs, lights = "", "", ""
    for g in range(4):
        uarts += (f"  - id: rs485_g{g + 1}\n    rx_pin: GPIO{16 + g}\n    baud_rate: 1000000\n"
                  f"    rx_buffer_size: 1024\n")
    for k in range(16):
        hubs += (f"  - id: port{k + 1}\n    parlio_id: ports\n    lane: {k}\n    num_boards: 2\n    telemetry:\n"
                 f"      uart_id: rs485_g{k // 4 + 1}\n      name: Port {k + 1}\n      receivers: 2\n"
                 f"      diagnostics: basic\n      direction_pin: GPIO{de[k]}\n      receiver_enable_pin:\n"
                 f"        mcp23xxx: rx_enable\n        number: {k}\n")
        if ports:
            hubs += f"    port: dp{k + 1}\n"
        lights += (f"  - platform: dig2analog_ultra\n    dig2analog_ultra_id: port{k + 1}\n    name: Port {k + 1} RGBCCT\n"
                   f"    board: 0\n    red: 1\n    green: 2\n    blue: 3\n    cold_white: 4\n    warm_white: 5\n")
    hubs += "  - id: rmt_line\n    pin: GPIO45\n    num_boards: 1\n    rmt_symbols: 48\n"
    lights += ("  - platform: dig2analog_ultra\n    dig2analog_ultra_id: rmt_line\n    name: RMT line\n    board: 0\n"
               "    channel: 1\n")
    expanders, diffpower = "", ""
    if ports:
        expanders = "  - id: port_enable\n    address: 0x21\n  - id: port_detect\n    address: 0x22\n"
        diffpower = "\ndiffpower:\n" + "".join(
            f"  - id: dp{k + 1}\n    name: Port {k + 1} power\n    enable_pin:\n      mcp23xxx: port_enable\n"
            f"      number: {k}\n    detect_pin:\n      mcp23xxx: port_detect\n      number: {k}\n      mode: INPUT\n"
            for k in range(16))
    return ("esphome:\n  name: d2a-ultra-p4-parlio" + ("-ports" if ports else "") + "\n\nesp32:\n  board: esp32-p4_r3\n"
            "  cpu_frequency: 360MHz\n  framework:\n    type: esp-idf\n\n"
            "external_components:\n  - source:\n      type: local\n      path: ../../../components\n"
            "    components: [dig2analog_ultra, dig2analog_ultra_parlio" + (", diffpower" if ports else "") + "]\n\n"
            "logger:\n\n"
            "i2c:\n  sda: GPIO43\n  scl: GPIO44\n\nmcp23017:\n  - id: rx_enable\n    address: 0x20\n" + expanders +
            "\nuart:\n" + uarts + diffpower +
            "\ndig2analog_ultra_parlio:\n  - id: ports\n    pins: [" + ", ".join(f"GPIO{i}" for i in range(16)) + "]\n"
            "\ndig2analog_ultra:\n" + hubs + "\nlight:\n" + lights)


def p4_parlio_lights_config() -> str:
    """tests/build/ultra/ultra-p4-parlio-lights.yaml: 8 lanes of a PARLIO bus on an ESP32-P4 at 10 MHz (8-bit samples,
    the automatic buffer time), lights only: no telemetry, no UART (the build without the direction pins' code)."""
    hubs, lights = "", ""
    for k in range(8):
        hubs += f"  - id: port{k + 1}\n    parlio_id: ports\n    lane: {k}\n    num_boards: 1\n"
        lights += (f"  - platform: dig2analog_ultra\n    dig2analog_ultra_id: port{k + 1}\n    name: Port {k + 1} RGBCCT\n"
                   f"    board: 0\n    red: 1\n    green: 2\n    blue: 3\n    cold_white: 4\n    warm_white: 5\n")
    return ("esphome:\n  name: d2a-ultra-p4-parlio-lights\n\nesp32:\n  board: esp32-p4_r3\n  cpu_frequency: 360MHz\n"
            "  framework:\n    type: esp-idf\n\nexternal_components:\n  - source:\n      type: local\n      path: ../../../components\n"
            "    components: [dig2analog_ultra, dig2analog_ultra_parlio]\n\nlogger:\n\n"
            "dig2analog_ultra_parlio:\n  - id: ports\n    sample_rate: 10MHz\n    pins: ["
            + ", ".join(f"GPIO{i}" for i in range(8)) + "]\n"
            "\ndig2analog_ultra:\n" + hubs + "\nlight:\n" + lights)


def telemetry_config(example: str) -> str:
    """tests/build/ultra/ultra-telemetry.yaml: ultra-example with Diff-Power receiver telemetry on its line: a
    device per receiver, the four lights of board 1 on their own devices (their estimates)."""
    src = example.replace("  name: d2a-ultra\n", "  name: d2a-ultra-telemetry\n", 1)
    assert "d2a-ultra-telemetry" in src
    src = src.replace("  friendly_name: ${friendly_name}\n", "  friendly_name: ${friendly_name}\n" + TELEMETRY_DEVICES, 1)
    src = src.replace("dig2analog_ultra:\n  id: ultra\n", "dig2analog_ultra:\n  id: ultra\n" + TELEMETRY_HUB, 1)
    src = src.replace("  signal_loss: hold ", "  signal_loss: scene", 1)   # (0.2.5: to the power-on scene)
    assert "  telemetry:\n" in src and "  devices:" in src and "  signal_loss: scene" in src
    for light, device in (("RGBCCT", "p1_r1"), ("Cabinet CCT", "cabinet"), ("Shelf 1", "shelf_1"),
                          ("Shelf 2", "shelf_2"), ("Shelf 3", "shelf_3")):
        old = f"    name: {light}\n    board:"
        assert src.count(old) == 1, light
        src = src.replace(old, f"    name: {light}\n    device_id: {device}\n    board:")
    return src.rstrip("\n") + "\n" + TELEMETRY_EXTRA


DIGITAL_CONFIG = """\
# tests/build/ultra/ultra-digital.yaml: digital ports without telemetry (the page 0x81 code compiled out): a line with one
# receiver and four segments, an ESPHome addressable light on the same strip (ultra_digital_stream, exclusive), the
# actions, a stored power-on effect; a second line with a WS2805 strip and the custom sliders
esphome:
  name: d2a-ultra-digital
esp32:
  board: esp32dev
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: ../../../components
    components: [dig2analog_ultra, ultra_digital, ultra_digital_stream]
logger:
api:
  encryption:
    key: !secret api_encryption_key
ota:
  - platform: esphome
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

dig2analog_ultra:
  - id: ultra
    pin: GPIO16
    num_boards: 1
    refresh_interval: 500ms
    digital:
      reset_time: 300us
  - id: ultra_bar
    pin: GPIO17
    num_boards: 1
    digital:

light:
  - platform: ultra_digital_stream
    id: streamer
    name: Streamer
    dig2analog_ultra_id: ultra
    num_leds: 120
    chipset: ws2812
    channel_colors: GRB
    effects:
      - pulse:
      - addressable_rainbow:
  - platform: ultra_digital
    id: seg0
    dig2analog_ultra_id: ultra
    name: Roof
    board: 0
    segment: 0
    chip: ws2812b
    num_leds: 120
    segment_length: 60
    exclusive_with: streamer
    store_after: 30s
    standalone:
      power_on: true
      on_signal_loss: true
  - platform: ultra_digital
    id: seg1
    dig2analog_ultra_id: ultra
    name: Roof back
    board: 0
    segment: 1
    chip: ws2812b
    num_leds: 120
    segment_start: 60
    reverse: true
    mirror: true
    lock: true
    controls: false
    status: false
  - platform: ultra_digital
    id: seg2
    dig2analog_ultra_id: ultra
    name: Roof edge
    board: 0
    segment: 2
    chip: ws2812b
    num_leds: 120
    segment_start: 100
    segment_length: 20
    catalogue_effects: false
    effects:
      - ultra_digital_fx:
          name: Mine
          fx: 130
      - random:
  - platform: ultra_digital
    id: bar
    name: Bar
    dig2analog_ultra_id: ultra_bar
    board: 0
    chip: ws2805
    num_leds: 90
    color_order: RGBWC
    cold_white_share: 30%
    auto_white: bright
    custom_controls: true
    led_current: 18mA
    current_limit: 3A
    max_fps: 60
    virtual_offset: 100
    virtual_length: 200

button:
  - platform: template
    name: Warm
    on_press:
      - ultra_digital.effect_params:
          id: seg0
          color_2: 0xFF8000
          color_3: [0, 64, 255]
          speed: 200
          intensity: 90
          custom1: 10
          custom2: 20
          palette: Ocean
  - platform: template
    name: Retry receiver
    on_press:
      - dig2analog_ultra.retry_receiver:
          id: ultra
"""


CHAIN_CONFIG = """# tests/build/ultra/ultra-chain.yaml: an Ultra-Digital receiver in chain mode, the last board of a normal Ultra line with
# telemetry (two Ultra-Analog receivers in front: lights as always; the Ultra-Digital one with two segments, its sliders, the
# Line status, the action), and, on a second line, a digital port with a receiver (the stream transport next to it)
esphome:
  name: d2a-ultra-chain
  areas:
    - id: garden
      name: Garden
  devices:
    - id: p1_r1
      name: Port 1 R1
    - id: p1_r2
      name: Port 1 R2
    - id: p1_r3
      name: Port 1 R3
      area_id: garden
esp32:
  board: esp32dev
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: ../../../components
    components: [dig2analog_ultra, ultra_digital, ultra_digital_stream]
logger:
api:
  encryption:
    key: !secret api_encryption_key
ota:
  - platform: esphome
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

uart:
  - id: rs485
    rx_pin: GPIO34
    baud_rate: 1000000
    rx_buffer_size: 1024

dig2analog_ultra:
  - id: ultra
    pin: GPIO16
    num_boards: 3
    rmt_symbols: 128
    telemetry:
      uart_id: rs485
      name: Port 1
      direction_pin: GPIO4
      receivers:
        - device_id: p1_r1
        - device_id: p1_r2
        - device_id: p1_r3
  - id: ultra_bar
    pin: GPIO17
    num_boards: 1
    digital:

light:
  - platform: dig2analog_ultra
    dig2analog_ultra_id: ultra
    name: Cove
    device_id: p1_r1
    board: 0
    cold_white: 1
    warm_white: 2
  - platform: dig2analog_ultra
    dig2analog_ultra_id: ultra
    name: Shelf
    device_id: p1_r2
    board: 1
    channel: 1
  - platform: ultra_digital
    id: roof
    dig2analog_ultra_id: ultra
    name: Roof
    device_id: p1_r3
    board: 2
    mode: chain
    segment: 0
    chip: sk6812_rgbw
    num_leds: 200
    segment_length: 100
    color_order: GRBW
    custom_controls: true
    store_after: 30s
    standalone:
      power_on: true
      on_signal_loss: true
  - platform: ultra_digital
    id: roof_edge
    dig2analog_ultra_id: ultra
    name: Roof edge
    device_id: p1_r3
    board: 2
    segment: 1
    chip: sk6812_rgbw
    num_leds: 200
    segment_start: 100
    reverse: true
    controls: false        # (the segments of one device share its entities: the first light has them)
    status: false
  - platform: ultra_digital
    id: bar
    name: Bar
    dig2analog_ultra_id: ultra_bar
    board: 0
    mode: auto
    chip: ws2812b
    num_leds: 90

button:
  - platform: template
    name: Warm
    on_press:
      - ultra_digital.effect_params:
          id: roof
          color_2: 0xFF8000
          speed: 200
          palette: Ocean
"""


def main() -> None:
    src = (ROOT / "examples" / "quinled-dig2analog-ultra-rgbcct.yaml").read_text(encoding="utf-8")
    src = src.replace(
        "  - source: github://intermittech/quinled-ultra-esphome@main\n    components: [dig2analog_ultra]",
        "  - source:\n      type: local\n      path: ../../../components\n    components: [dig2analog_ultra]",
    )
    src = src.replace("  # boot_protocol: ultra", "  boot_protocol: ultra  #")
    # a supply on the line: the firmware update's power hold / power cycle compile too
    src = src.replace("dig2analog_ultra:\n  id: ultra\n", "dig2analog_ultra:\n  id: ultra\n  power_supply: led_psu\n", 1)
    assert "  power_supply: led_psu\n" in src
    src = src.replace("    # power_on_scene: true", "    power_on_scene: true  #")
    # the hub's own calibration schedule (off in the example: calibration is a manual step)
    src = src.replace("  power_supply: led_psu\n", "  power_supply: led_psu\n  current_calibration:\n    interval: 7d\n"
                      "    only_when_off: true\n", 1)
    assert "  current_calibration:\n    interval: 7d\n" in src
    src = src.replace("    # dim_to_warm:", "    dim_to_warm:  #")
    src = src.replace("    id: rgbcct\n", "    id: rgbcct\n    output_id: rgbcct_out\n", 1)
    assert "output_id: rgbcct_out" in src
    src = src.replace("    #   warm_color_temperature: 2000 K", "      warm_color_temperature: 2000 K")
    src = src.replace("    #   red_assist: 25%", "      red_assist: 25%  #")
    assert "path: ../../../components" in src and "  boot_protocol: ultra  #" in src
    assert "    power_on_scene: true  #" in src
    # the wiki's PWM frequency select replaces the example's own (same name)
    src, n = re.subn(r"  - platform: template\n    name: PWM frequency\n(?:    .*\n|      .*\n)*?    id: pwm_select\n", "", src)
    assert n == 1, "example PWM select"
    src = src.replace("\nselect:\n", "\nselect:\n" + HA_SELECT, 1)
    # dim to warm set points on the example's CCT light
    old = "    cold_white_color_temperature: 5000 K\n    warm_white_color_temperature: 2700 K\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    dim_to_warm:\n      points:\n        - brightness: 100%\n"
                      "          color_temperature: 4500 K\n        - brightness: 20%\n          color_temperature: 3000K\n"
                      "        - brightness: 1%\n          color_temperature: 370 mireds\n")
    # power-on effects (SCENE_EFFECT): the CCT light with values (a colour temperature), Shelf 3 by the effect's name
    old = "    effects:\n      - ultra_breathe:\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    power_on_effect:\n      effect: Breathe\n      brightness: 40%\n"
                      "      color_temperature: 3000 K\n")
    old = "      - ultra_candle:\n          intensity: 30%\n"
    assert src.count(old) == 1
    src = src.replace(old, old + "    power_on_effect: Candle\n")
    assert "set_stagger(-1, i)" in src
    # the example's own button: list gets the test buttons appended
    src = src.replace("\nbutton:\n", "\nbutton:\n" + EXTRA.split("button:\n", 1)[1], 1)
    src += EXTRA.split("\nbutton:\n", 1)[0] + "\n"
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "ultra-example.yaml").write_text(src, encoding="utf-8")
    (OUT / "ultra-strip.yaml").write_text(strip_config(), encoding="utf-8")
    (OUT / "ultra-telemetry.yaml").write_text(telemetry_config(src), encoding="utf-8")
    (OUT / "ultra-p4-telemetry.yaml").write_text(p4_telemetry_config(), encoding="utf-8")
    (OUT / "ultra-diffpower.yaml").write_text(DIFFPOWER_CONFIG, encoding="utf-8")
    (OUT / "ultra-digital.yaml").write_text(DIGITAL_CONFIG, encoding="utf-8")
    (OUT / "ultra-chain.yaml").write_text(CHAIN_CONFIG, encoding="utf-8")
    (OUT / "ultra-p4-parlio.yaml").write_text(p4_parlio_config(), encoding="utf-8")
    (OUT / "ultra-p4-parlio-lights.yaml").write_text(p4_parlio_lights_config(), encoding="utf-8")
    (OUT / "ultra-p4-parlio-ports.yaml").write_text(p4_parlio_config(ports=True), encoding="utf-8")
    secrets = OUT / "secrets.yaml"
    have = secrets.read_text(encoding="utf-8") if secrets.exists() else ""
    need = (
        'wifi_ssid: "test"\nwifi_password: "testtest"\n'
        'api_encryption_key: "AQIDBAUGBwgJCgsMDQ4PEBESExQVFhcYGRobHB0eHyA="\n'
        'fallback_ap_password: "testtest"\n'
    )
    for line in need.splitlines():   # (the examples' secrets: any missing added)
        if line.split(":")[0] + ":" not in have:
            have += ("" if not have or have.endswith("\n") else "\n") + line + "\n"
    secrets.write_text(have, encoding="utf-8")
    print(f"written {OUT / 'ultra-example.yaml'}, {OUT / 'ultra-strip.yaml'}, {OUT / 'ultra-telemetry.yaml'}, "
          f"{OUT / 'ultra-p4-telemetry.yaml'}, {OUT / 'ultra-diffpower.yaml'}, {OUT / 'ultra-p4-parlio.yaml'}, "
          f"{OUT / 'ultra-p4-parlio-lights.yaml'}, {OUT / 'ultra-p4-parlio-ports.yaml'}")


if __name__ == "__main__":
    main()
