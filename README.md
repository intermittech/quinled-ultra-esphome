# quinled-ultra-esphome

ESPHome external components for boards on the **QuinLED Ultra protocol**, such as the **[Dig2Analog-Plus-5ch](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Dig2Analog-Plus-5ch)**
and the **[Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M)**, in **Ultra mode**. The board works as a co-processor: ESPHome sends
colours and transition times; analog receivers interpolate them every millisecond in 16 bits. Analog effects run
on ESPHome and sample at most 20 times per second per light, with receiver fades between samples. They share 30% of
the line's slot capacity; the hub scales their rates together when necessary and coalesces queued samples.
[Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) pixel effects continue to run on their receiver.

| Board | | |
|---|---|---|
| Dig2Analog-Plus-5ch | Ultra mode: `ultra_line` (this repository) | In development |
| Diff-Power-UR-Analog-M | `ultra_line` with telemetry | In development |
| Diff-Power-UR-Digital-M | `ur_digital_m` / `ur_digital_m_stream` | In development |

See the [board overview](https://github.com/intermittech/quinled-ultra-firmware/wiki/Boards).

| Component | |
|---|---|
| `ultra_line` | The data line (hub), the `ultra_line` light platform with host effects, board settings, current calibration and board firmware updates |
| `ultra_analog_strip` | ESPHome's addressable effects (rainbow, scan, twinkle, ...) across Ultra lights, one light per pixel |
| `diffpower` | QuinLED Diff-Power sender ports: 55 V only while a light on the port needs it (standby at 4.5 V, board detection, pre-wake, the dark window), for any ESPHome light |
| `ur_digital_m` | A light per segment of a Diff-Power-UR-Digital-M receiver's pixel strip, whose effects the receiver makes itself (the catalogue's effect list, Speed / Intensity / Palette / Colour 2 / 3 / Custom 1 / 2, transitions on the receiver; exclusive with a streaming light on a digital port; `mode: chain` at any position of a normal Ultra line) |
| `ur_digital_m_stream` | An ESPHome addressable light on the strip behind a receiver: its pixel frames go through the digital port's hub (`ultra_line` with `digital:`), between the line's marker frames, and stop while the receiver shows a local effect |
| `ultra_parlio` | Up to 16 data lines from the ESP32-P4's PARLIO TX unit (one DMA stream) instead of the RMT's 4 |

## Install

The `ur_digital_m` light validates `num_leds` per receiver by `board_type`: at most **600 RGB / 500 RGBW / 400 RGBCCT** on
the V203, **800 / 600 / 500** on the V407.
`max_fps` defaults to **60**. Higher values up to 1000, or 0 for back-to-back frames, are allowed; complete wire frames
and the chip reset always limit the actual rate. Settings shared by segments apply to their whole receiver.

Each `ur_digital_m` light has **Speed**, **Intensity**, **Palette**, **Colour 2** and **Colour 3** controls by default
(`controls: false` omits them), plus **Custom 1** and **Custom 2** (`custom_controls: false` omits those).
The controls keep their values across restarts. Every effect takes the **Palette**; it stays as chosen when the effect
changes. **Effect default** gives each effect its own look (its default palette, or its own colours), **Colors 1-2-3**
builds the palette from the three colours. Colour 2 / 3 are configuration text entities: enter `#RRGGBB`
(lower-case hex is accepted and shown in upper case). Their first values come from `color_2` / `color_3` in YAML
unless stored values exist; `ur_digital_m.effect_params` colour changes also update and store the text values.

Set `board_type: v203` (the default) or `board_type: v407` on each `ur_digital_m` light; all segments of one receiver
must agree. This chooses the catalogue used for Home Assistant's native effect list and Palette select. V203 keeps
36 effects / 31 palettes; V407 has the approved 114 effects (ids 1–114) / 49 palettes (ids 0–48).
`pixel_fx_catalogue_v407.json` has the same format as the V203 `pixel_fx_catalogue.json` and retains
the new effects' `design` data and palettes' `stops`/`mood`. Each effect lists `default_palette_id` (or
`default_palette: "own"`), `palette_slots` (the colours it still uses with a palette) and `palette_note`.
Its integer `catalogue_version` is 4 (as the V203 file); `board_catalogue_version: "4+v407-1+pal49"` keeps the board revision.

Each receiver/device also publishes **Effect capabilities**, an enabled diagnostic text sensor shared by its segment lights
(keep it enabled for card discovery).
Its JSON is `{v:1, source:"build"|"runtime", board:"v203"|"v407", effects:"<48 hex chars>", palettes:"<48 hex chars>"}`.
Each map is 24 bytes, exactly page 0x11: bit `(id & 7)` of byte `(id >> 3)` means id 0–191 is supported;
palette ids 254/255 are always available. Before telemetry, and while either map is incomplete, it publishes the
configured board's maps. Existing page 0x11 polling reads both once after start and rereads them after a receiver
restart/replacement; completed maps replace the diagnostic state only when changed. One-way lines retain `source: build`.
The diagnostic category keeps this metadata out of normal light controls; disabling the entity removes runtime filtering.

```yaml
external_components:
  - source: github://intermittech/quinled-ultra-esphome@main
    components: [ultra_line, dig2analog_plus]
```

## Quick start

Two chained boards: an RGBCCT light on the first, a single-colour light on the second.
Protocol 0.3 requires a validated descriptor for each board. Telemetry discovers descriptors automatically;
one-way lines need exported descriptor files in `descriptors:` (a list of `board:` and `file:` entries), or on a
pin of one board model `every_board_descriptor:` (one file for every board; a board's own entry wins).
Normal commands wait for valid inventory. Obtain the files from the matching firmware or provisioning tool.

```yaml
ultra_line:
  id: ultra
  pin: GPIO16            # to the first board's Data-In
  num_boards: 2          # boards on this line, up to 50

light:
  - platform: dig2analog_plus
    name: RGBCCT
    board: 1              # 1 = the first board after the controller (counted like num_boards)
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
  - platform: dig2analog_plus
    name: Shelf
    board: 2
    channel: 1
```

## What Ultra mode adds over WS2805 / WS2915

| | WS2805 / WS2915 boards ([esphome-ws2915-ws2805](https://github.com/intermittech/esphome-ws2915-ws2805)) | QuinLED Dig2Analog-Plus-5ch in Ultra mode |
|---|---|---|
| Resolution | 8 bit (WS2805) or 16 bit (WS2915) frames | 16 bit with an exact gamma curve on the board, no 8-bit stage |
| PWM | the chip's own: ~4 kHz (WS2805), ~2 kHz (WS2915) | 2.2-100 kHz, the channels staggered, spread spectrum |
| Transitions | ESPHome sends every step, as smooth as the frame rate | the board recomputes every 1 ms, in step on every board |
| Effects | streamed from ESPHome, frame by frame | on ESPHome (pulse, strobe, flicker, random, candle, breathe, colour loop, sunrise, sunset); receiver fades interpolate the steps |
| Also | | stored power-on level, host configuration for a separate signal-loss level, dim to warm, board current sense and calibration (on the board: protection, limits, automatic dimming), firmware updates over the data line |

**Reporting back** (voltage, current, power, energy, temperature, power events and why a light is dim or off, to
ESPHome and Home Assistant) is for **Diff-Power Ultra Receiver boards only**: the line's `telemetry:` ([Diff-Power
telemetry](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-Telemetry)). The QuinLED Dig2Analog-Plus-5ch's data line is one-way: it measures for itself and never
reports back.

The QuinLED Dig2Analog-Plus-5ch can also run in WS2805 / WS2915 mode with the
[`ws2915`](https://github.com/intermittech/esphome-ws2915-ws2805) component (then it already has its own PWM).

## Documentation

See the **[wiki](https://github.com/intermittech/quinled-ultra-esphome/wiki)**:

| | |
|---|---|
| [Ultra mode](https://github.com/intermittech/quinled-ultra-esphome/wiki/Ultra-Mode) | What it is, a quick start and its pages: lights, effects, board settings, current, line options, firmware updates, how it works |
| [Ultra strip effects](https://github.com/intermittech/quinled-ultra-esphome/wiki/Ultra-Strip) | `ultra_analog_strip`: ESPHome's addressable effects across Ultra lights |
| [Diff-Power ports](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-Ports) | `diffpower`: a Diff-Power sender's ports in standby, waking; [power modes](https://github.com/intermittech/quinled-ultra-esphome/wiki/Power-Modes), pre-wake |
| [Diff-Power telemetry](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-Telemetry) | Diff-Power Ultra Receiver boards reporting back: entities, dashboards, the lifetime ledger |
| [Diff-Power-UR-Digital-M effects](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-UR-Digital-M-Effects) | `ur_digital_m`: local pixel effects on the Diff-Power-UR-Digital-M |
| [Diff-Power-UR-Digital-M port](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-UR-Digital-M-Port) | `digital:` on the line: the receiver in a pixel strip's data path, frames behind markers, the stream light, polls and the back-off |
| [Diff-Power-UR-Digital-M chain mode](https://github.com/intermittech/quinled-ultra-esphome/wiki/Diff-Power-UR-Digital-M-Chain) | `mode:` of `ur_digital_m`: the receiver at any position of a normal Ultra line, next to Diff-Power-UR-Analog-M receivers, the Line status |
| [Installation](https://github.com/intermittech/quinled-ultra-esphome/wiki/Installation) | Adding the components, pinning a version |
| [Development](https://github.com/intermittech/quinled-ultra-esphome/wiki/Development) | Tests and layout |

Ready-made configurations are in [`examples/`](examples).

**Related:**
- [quinled-ultra-firmware](https://github.com/intermittech/quinled-ultra-firmware): the board firmware.
- [quinled-ultra-protocol](https://github.com/intermittech/quinled-ultra-protocol): the QuinLED Ultra protocol
  (`components/ultra_line/ultra_protocol.h` is its reference header, unchanged).

## Status

Compiles and passes its tests for the ESP32 classic (ESP-IDF). The QuinLED Dig2Analog-Plus-5ch is still in
development; not yet verified on real hardware.

## License

GPL-3.0, the same as the ESPHome C++ core these components compile into. See [LICENSE](LICENSE). The vendored QuinLED
Ultra protocol header and sender library keep their MIT licence; palette credits (FastLED, WLED, cpt-city) are in
[NOTICE.md](NOTICE.md).

Analog lights retain the `ultra_*` YAML effect names. Pulse, random and strobe use ESPHome's built-in effects;
`ultra_flicker` uses its flicker generator at the bus cadence. Candle, sine breathe, hue loop, sunrise and sunset
are component effects on the hub. Normal transitions (including brightness or colour ramps) use SET directly.
Native pulse, random and strobe retain their endpoint timings. Sampled effects use `effect_rate:` on the hub
(default 20, range 1–20), with an optional per-light override. Each sample's fade lasts until the next sample.

Normal lines use the 800 kbit/s profile and at least 500 µs of processing LOW, extended by the descriptors.
A Diff-Power port takes at most 8 boards and a Dig2Analog-Plus-5ch data pin 50: the line's `port_type:` (`diff_power`
or `d2a_pin`; not given: `diff_power` with telemetry, `port:`, `digital:` or a Diff-Power Ultra Receiver light).
A pin's board count picks its speed:
1–20 boards 800, 21–34 600, 35–50 200 kbit/s. Above 20 boards it needs a complete all-Ultra inventory
(`inventory_complete: true`, `bootloader_profile_mask` with that profile, a `descriptors:` file for every board
or `every_board_descriptor:`).
A pin never steps down on its own; `initial_profile:` can only make it slower. If a board's firmware lacks the profile,
the line stays at 800, logs an error and shows why in the optional `text_sensor` `line_speed:`.
`line_rate:` uses a fresh, coherent LINK_STATS `RATE_REQUEST` to step down one profile and retrain the whole chain.
For receivers without that flag, consecutive windows of excessive CRC errors provide the fallback.
Each receiver has a `rate_request` diagnostic, disabled by default. On a Diff-Power port slower profiles require
`inventory_complete: true`, a common application/bootloader profile mask and a qualified `path_distortion_ns`.
The hub never raises the rate automatically. PARLIO lanes sharing a DMA group change rate together.

The update entity is "Update all receivers on this port": its plan gives every board the release's image for its
exact immutable target, one MCU family after the other, with a reason for every board that gets none. Receivers with a
return channel report their target themselves; list every other board under `inventory:` with its protected boot
identity: `boot_identity:` (a shipped bootloader's by name, e.g. `dig2analog_plus_5ch_rev04_bl4`: no file) or the jig's `boot_info:`
file (optional layout and `blackout:`; otherwise the signed manifest's). A Diff-Power port holds at most 8
boards and a Dig2Analog-Plus-5ch data pin 50 (the line's `port_type:`). Results per board stay in the optional `report:` sensor:
verified needs fresh identity and FW_UPDATE evidence, a one-way board ends sent, unverified. Install runs the port
job: every image is broadcast three times in full, then each receiver's version is read back. While a known receiver
doesn't answer, Install refuses and names it ("B2 not answering: check power and cable"). Each receiver then has a
**Recover firmware** button: it sends that receiver its image blind, from the identity it last reported (kept on the
ESP32); verified when it is found again, else "recovery sent".

ESPHome's own web page (web_server v3) showed no Install button for this entity on the first bench run: add a template
`button:` whose `on_press` runs `update.perform: <update id>` (`examples/quinled-dig2analog-plus-line-update.yaml`);
`http_request:` needs `buffer_size_rx: 2048` for GitHub's answer.

Use `power_on_level:` for fixed stored brightness/colour values with optional `fade:` (default 300 ms, maximum 1 s).
For example `power_on_level: 60%` stores a steady level. `power_on_scene: true` follows the light's restored values.
`signal_loss_level:` stores a separate level selected by `signal_loss: scene`. Both levels fade in on the receiver.
## Diff-Power-UR-Digital-M calibration

With protocol 0.3 and telemetry, the receiver's **Calibrate current** button also detects addressable pixels.
HA shows configured/detected/used counts, a mismatch notice, phase/result and a digital v2 Power model.
See the [calibration wiki](https://github.com/intermittech/quinled-ultra-esphome/wiki/Ultra-Current) for the manual run and current model contract.
