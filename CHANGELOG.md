# Changelog

A summary of the changes to the ESPHome components for QuinLED Ultra boards, by the protocol release they follow.
This repository holds the current state only.

## Protocol 0.3.2, 2026-10-31

- **Board positions count from 1**, like `num_boards`: board 1 is the first board after the controller. A light's
  `board:`, the update inventory, `descriptors:`, `current_calibration: boards:`, the actions `calibrate_current` /
  `show_version` and the lambdas take 1 .. `num_boards`; `board: 0` and a board beyond `num_boards` are refused with
  a message. Every log line, report and status text names the boards the same way (`B1`, `Receiver 1`).
- `pulse_skip_rate: 2kHz` (the default) or `1kHz`: how slow the pulse-skipping patterns below the minimum pulse may
  repeat (lower dimming at 1 kHz); sent only to boards that support it. `min_pulse` left out = each board's own
  default (Dig2Analog-Plus-5ch rev 0.4 100 ns, others 200 ns).
- `max_channel_current` per channel is at most that channel's own ceiling on the line's boards (Dig2Analog-Plus-5ch
  rev 0.4: R / G / B 3 A, W1 / W2 5 A).
- Update: every decision frame of the update sender starts at least 10 ms after the frame before it (a board's app
  dropped an announcement that came too soon after a PING, and the update never started).
- The log: `dump_config` shows the slowest pulse-skip rate as the board plays it (below 4 kHz it showed twice the real
  rate) and a custom `pwm_frequency` as set (it showed 1.44 times the value).
- Update: a board without a return channel names its bootloader in the inventory (`boot_identity:
  dig2analog_plus_5ch_rev04_bl4`, built into the component: no file) or keeps the `boot_info:` file; one of the two.
- Update progress counts `pass 1 of 3` .. `pass 3 of 3` (it showed one more).
- Current calibration on boards without a return channel waits 1,270 ms per board (the firmware's 30 ms steps); a
  test the board did not take shows `Not taken: no load` / `Not taken: no change`.
- Example `quinled-dig2analog-plus-line-update.yaml`: the update with an Install button for ESPHome's web page and
  `http_request: buffer_size_rx: 2048` (GitHub's response headers).
- Ultra protocol 0.3.2-draft: the vendored sender sends a blind update's closing frame (END) three times and the idle
  confirmation as six session-zero PINGs (an app confirms from the fourth).
- `diffpower`: a wake timeline per port (`wake duration`, `wake timeline`, `wake warning`): from the enable to ready,
  the first lit frame and the first reply, in ms; a warning when a wake takes over 2.5 s or the port drops while it starts.
- Telemetry: a cable-drop advisor per receiver (`Cable drop`, `Full-load voltage`, `Full-load drop`); a supply dip that
  follows a load step and that the cable explains is labelled "cable drop (expected)".
- Telemetry: port load rules per port (`<port> load rules`, `<port> load rules problem`): at most 4 receivers at 36 V
  per port, and a 36 V receiver that would get less than 40 V at full load (cable up to 50 m on 48 V, 100 m on 55 V).
- Telemetry: on a board without supply-dip records (the CH32V203 Digital-Medium) a supply dip is a note, a Problem
  only when its bus reading fell below `dip_warning` % of its highest.
- A board without a return channel and without a descriptor in the YAML (a Dig2Analog-Plus-5ch pin up to 20 boards)
  gets its light commands; before, its slots stayed empty and the board dark.
- The log (tag `ultra_line`) shows each light change per board as the frame carries it, when it is out, and why a
  change can't go out (an update, a port that is off, no descriptor, ...).

## Protocol 0.3.1, 2026-10-27

- Ultra protocol 0.3.1-draft (wire VERSION 3): the protocol header and the sender library vendored byte-identical;
  the shared 200 µs slot-plan re-arm and the PIXEL BUSY counter.
- `ur_digital_m`: the Diff-Power Ultra Receiver Digital-Medium at any position of a normal Ultra line, several per
  line, mixed with the other receivers up to the port type's board limit; it relays the answers of the boards behind
  it. STREAM stays alone on a digital port.
- `ur_digital_m` on the CH32V407 (`board_type: v407`): `num_leds` up to 800 RGB / 600 RGBW / 500 five-channel pixels,
  and a pixel calibration up to the board's own limits from its descriptor (the CH32V203 keeps 600 / 500 / 400).
- `ur_digital_m`: a pixel calibration waits for the receiver's own deadline from its descriptor (the protocol's
  50 ms + 1.5 x `cal_ms`: 9.05 s on the CH32V203, 9.8 s on the CH32V407), not a fixed 6 s.
- `diffpower`: a port in Always on is checked again once all its lights are dark: a planned power-off of at least
  2 s, once per dark period.

## Protocol 0.3, 2026-10-24

- Ultra protocol 0.3 (wire VERSION 3): the protocol header and the C99 sender library vendored byte-identical from
  quinled-ultra-protocol; CI checks the copy.
- "Update all receivers on this port": one update entity per port, a manifest matched per exact target, a per-board
  plan, blind group broadcasts per chip family with three complete DATA passes, verification from fresh identities
  only, and a per-board report. A known receiver that doesn't answer blocks the job and is named; a Recover
  firmware button sends that board its image.
- Management traffic (updates, commissioning, descriptor and LIFE_SYNC pages) waits until the receivers have started:
  their first reply in this power-on, at most 5 s after port-up on a silent return channel (a warning), 500 ms
  from bus-on on a line without replies; light and scene frames keep their schedule. The update keeps its full
  preamble after that wait; a silent known receiver still blocks "update all".
- LIFE_SYNC: the lifetime energy step sent to a receiver is the protocol's allowance from its descriptor (input
  rating x maximum voltage); an unknown input sends no energy step.
- One shared start lands on the same receiver tick on every board: header TIME wraps on a whole ms (protocol 0.3).
- Line speed per port type: up to 8 receivers on a Diff-Power port, up to 50 boards on a Dig2Analog-Plus-5ch pin with a
  speed table by board count; `port_type` on the hub line, an optional line speed sensor.
- Board names: `ur_digital_m` / `ur_digital_m_stream` for the Diff-Power Ultra Receiver Digital-Medium; the
  CH32V407 pixel effect catalogue.
- Component names: `ultra_line` (the line hub for every board: its actions, sensors and update entity), the light
  platforms `dig2analog_plus` (QuinLED Dig2Analog-Plus-5ch) and `ur_analog_m` (Diff-Power Ultra Receiver
  Analog-Medium), `ultra_analog_strip` and `ultra_parlio`.
- Every pixel effect takes a palette (catalogue version 4): Effect default is the effect's own look, each effect
  notes what the palette colours, and the Palette select keeps its choice when the effect changes.
- Telemetry: the Supply impedance comes from the ENERGY page on every receiver. A receiver whose features leave out
  POWER events, SUPPLY dips or DIAG (the Diff-Power Ultra Receiver Digital-Medium on the CH32V203) is never asked
  for them; their entities stay unknown there.
- NOTICE.md: the vendored MIT sources and the palette credits.
- Tests: the receiver contracts and the cross-repo checks run by default against the pinned protocol and firmware
  (`tests/check_pins.py`, `tests/run_all.py`).

## Protocol 0.2.9, 2026-10-17

- `ur_digital_m` chain mode: a receiver as the last board of a normal Ultra line; `pixel_smoothing`; the catalogue's
  version 3 (36 effects, 31 palettes); RW frames end with a held LOW before the line is released.
- Settings from Home Assistant (controller, ports, receivers); a per-line setup info text for the setup generator.

## Protocol 0.2.8, 2026-10-17

- Lights for the Digital-Medium receiver's local pixel effects; the digital port's hub transport (markers, latch
  gaps, polls, back-off) and the streaming light.

## Protocol 0.2.7, 2026-10-16

- Commissioning per line with a plain-language report; current calibration from Home Assistant; the power mode Off
  per port; health per line and a slower reply rate on a bad cable.

## Protocol 0.2.6, 2026-10-15

- The `diffpower` component: QuinLED Diff-Power sender ports with standby, waking and the dark window.
- The receivers' lifetime ledger on the controller (LIFE_SYNC); PARLIO lanes for the ESP32-P4 (up to 16 lines).
- Size and CPU passes: no allocation per response window, compact state.

## Protocol 0.2.5, 2026-10-15

- Receiver telemetry in Home Assistant, load per port, automatic retry of board updates.

## Protocol 0.2.2 - 0.2.4, 2026-10-13 / 14

- The first ESPHome components for QuinLED Ultra boards; power-on effects; board firmware updates over the data
  line; synchronized group start; diagnostics; CI.
