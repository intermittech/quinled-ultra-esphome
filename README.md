# quinled-ultra-esphome

ESPHome external components for **QuinLED Ultra** boards, such as the **QuinLED dig2analog-ultra-5ch**, in
**Ultra mode**. The board works as a co-processor: ESPHome sends **what** should happen (a colour with a transition
length, an effect, a setting), and the board computes **how**, every millisecond, in 16 bit, in step with every
other board on the line.

| Board | | |
|---|---|---|
| QuinLED dig2analog-ultra-5ch | Ultra mode: `dig2analog_ultra` (this repository) | In development |

| Component | |
|---|---|
| `dig2analog_ultra` | The data line (hub), the `dig2analog_ultra` light platform with board effects, board settings, current calibration and board firmware updates |
| `dig2analog_ultra_strip` | ESPHome's addressable effects (rainbow, scan, twinkle, ...) across Ultra lights, one light per pixel |

## Install

```yaml
external_components:
  - source: github://intermittech/quinled-ultra-esphome@main
    components: [dig2analog_ultra]
```

## Quick start

Two chained boards: an RGBCCT light on the first, a single-colour light on the second.

```yaml
dig2analog_ultra:
  id: ultra
  pin: GPIO16            # to the first board's Data-In
  num_boards: 2          # boards on this line, up to 50

light:
  - platform: dig2analog_ultra
    name: RGBCCT
    board: 0
    red: 1
    green: 2
    blue: 3
    warm_white: 4
    cold_white: 5
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2700 K
    color_interlock: true
    effects:
      - ultra_candle:
      - ultra_color_loop:
  - platform: dig2analog_ultra
    name: Shelf
    board: 1
    channel: 1
```

## What Ultra mode adds over WS2805 / WS2915

| | WS2805 / WS2915 boards ([esphome-ws2915-ws2805](https://github.com/intermittech/esphome-ws2915-ws2805)) | QuinLED dig2analog-ultra-5ch in Ultra mode |
|---|---|---|
| Resolution | 8 bit (WS2805) or 16 bit (WS2915) frames | 16 bit with an exact gamma curve on the board, no 8-bit stage |
| PWM | the chip's own: ~4 kHz (WS2805), ~2 kHz (WS2915) | 2.2-100 kHz, the channels staggered, spread spectrum |
| Transitions | ESPHome sends every step, as smooth as the frame rate | the board recomputes every 1 ms, in step on every board |
| Effects | streamed from ESPHome, frame by frame | on the board (pulse, strobe, flicker, random, candle, breathe, colour loop, sunrise, sunset); ESPHome's own effects glide |
| Also | | power-on scene, dim to warm, board current sense and calibration (on the board: protection, limits, automatic dimming), firmware updates over the data line |

**Reporting back** (currents, temperature, voltages, calibration results, firmware versions to ESPHome and Home
Assistant) is for **QuinLED Diff-Power ultra receivers only**. The QuinLED dig2analog-ultra-5ch's data line is
one-way: it measures for itself and never reports back.

The QuinLED dig2analog-ultra-5ch can also run in WS2805 / WS2915 mode with the
[`ws2915`](https://github.com/intermittech/esphome-ws2915-ws2805) component (then it already has its own PWM).

## Documentation

See the **[wiki](https://github.com/intermittech/quinled-ultra-esphome/wiki)**:

| | |
|---|---|
| [Ultra mode](https://github.com/intermittech/quinled-ultra-esphome/wiki/Ultra-Mode) | Lights, board effects, board settings, line options, firmware updates, how it works |
| [Ultra strip effects](https://github.com/intermittech/quinled-ultra-esphome/wiki/Ultra-Strip) | `dig2analog_ultra_strip`: ESPHome's addressable effects across Ultra lights |
| [Installation](https://github.com/intermittech/quinled-ultra-esphome/wiki/Installation) | Adding the components, pinning a version |
| [Bench plan](https://github.com/intermittech/quinled-ultra-esphome/wiki/Bench-Plan) | What still needs checking on hardware |
| [Development](https://github.com/intermittech/quinled-ultra-esphome/wiki/Development) | Tests and layout |

Ready-made configurations are in [`examples/`](examples).

**Related:**
- [quinled-ultra-firmware](https://github.com/intermittech/quinled-ultra-firmware): the board firmware.
- [quinled-ultra-protocol](https://github.com/intermittech/quinled-ultra-protocol): the protocol
  (`components/dig2analog_ultra/ultra_protocol.h` is its reference header, unchanged).

## Status

Compiles and passes its tests for the ESP32 classic (ESP-IDF). The QuinLED dig2analog-ultra-5ch is still in
development; see the [bench plan](https://github.com/intermittech/quinled-ultra-esphome/wiki/Bench-Plan).

## License

GPL-3.0, the same as the ESPHome C++ core these components compile into. See [LICENSE](LICENSE).
