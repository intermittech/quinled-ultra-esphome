# Wiki draft: PARLIO lanes and shared telemetry UARTs

Text for the quinled-ultra-esphome wiki (the wiki clone is shared, so it lives here until it is merged). Three parts:
the new Ultra-Mode section, the changes to "Several lines with telemetry", and the Development page.

---

## 1. Ultra-Mode.md: new section after "Several lines with telemetry"

### PARLIO (ESP32-P4, up to 16 lines)

The RMT gives an ESP32-P4 four data lines. Its PARLIO TX unit drives up to **16 data pins from one DMA stream**:
`dig2analog_ultra_parlio:` is that bus, and a line on it says `parlio_id:` and `lane:` instead of `pin:`. Everything
else stays the line's own: its boards and settings, lights, start delay, preamble, refresh, telemetry and firmware
updates. Lanes and RMT lines can be mixed (the lanes don't use the RMT). The P4 has one PARLIO TX unit: one bus. The
ESP32-S31 (16-bit PARLIO in its preliminary datasheet) follows once ESP-IDF supports it.

```yaml
external_components:
  - source: github://intermittech/quinled-ultra-esphome@main
    components: [dig2analog_ultra, dig2analog_ultra_parlio]

dig2analog_ultra_parlio:
  - id: ports
    pins: [GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7,
           GPIO8, GPIO9, GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, GPIO15]   # lane 0, 1, ... 15
    # sample_rate: 20MHz            # the defaults
    # buffer_time: 1ms
    # psram: false

uart:                               # 4 UARTs, each shared by 4 ports
  - id: rs485_g1
    rx_pin: GPIO16
    baud_rate: 1000000
    rx_buffer_size: 1024
  # rs485_g2 .. g4 on GPIO17 .. 19

dig2analog_ultra:
  - id: port1
    parlio_id: ports
    lane: 0
    num_boards: 8
    telemetry:
      uart_id: rs485_g1             # ports 1-4
      name: Port 1
      direction_pin: GPIO20         # the port's RS-485 driver enable
      receiver_enable_pin: GPIO43   # its receiver's output on the shared RX (see below)
      receivers: 8
  - id: port2
    parlio_id: ports
    lane: 1
    num_boards: 8
    telemetry:
      uart_id: rs485_g1
      name: Port 2
      direction_pin: GPIO21
      receiver_enable_pin: GPIO44
      receivers: 8
  # ... port 16 on lane 15
```

| Option | Default | |
|---|---|---|
| `pins` | – | 1-16 data pins, lane 0 first; a line's `lane:` is its index here (`inverted: true` per pin works) |
| `sample_rate` | 20MHz | the sample clock: 20 MHz (50 ns) keeps the default bit timing exact; 10 MHz halves the buffers, the '0' becomes 300 ns (see below) |
| `buffer_time` | 1ms | one of the two DMA buffers (0.25-10 ms): a frame starts up to ~3 of them after it was queued; a longer one rides out a busier CPU, a shorter one saves RAM |
| `psram` | false | the two buffers in PSRAM (internal RAM by default; the other one when it has no room) |

**How it works:**

- **One stream for every line.** Each lane's waveform is one bit of every sample (16-bit samples; 8-bit with up to 8
  pins). Two DMA buffers take turns: while one goes out, a task renders the next ~1 ms into the other, with every
  lane's frames, resets and response windows. No refill interrupts; each line still sends its frames at its own
  times, as on the RMT.
- **Bit timing on the sample grid.** Every bit of the bus takes the same time: the longest bit of its lines (the
  '1''s 1.35 µs by default), so the shorter bit's low time is stretched. The high times, which carry the bits, stay
  each line's own. The configuration check rounds them to the sample clock and refuses what leaves the protocol's
  Physical Layer:

| | '0' high / low | '1' high / low | a bit |
|---|---|---|---|
| YAML defaults (RMT) | 250 / 1050 ns | 700 / 650 ns | 1.30 / 1.35 µs |
| PARLIO, 20 MHz (default) | 250 / **1100** ns | 700 / 650 ns | 1.35 µs |
| PARLIO, 10 MHz | **300** / **1100** ns | 700 / **700** ns | 1.40 µs |
| Physical Layer | 220-380 / 450-10000 ns | 580-2000 / 450-10000 ns | |

  At 20 MHz only the '0''s low changes (+50 ns: frames ~2 % longer than on the RMT). At 10 MHz the '0' is 300 ns: 50
  ns off its centre on the boards' 472 ns decision, which long chains (50 boards) may feel; 20 MHz keeps 250 ns.
- **Frames start on a group of 8 bits** (10.8 µs at 20 MHz) and the reset is rounded up to whole groups (302.4 µs).
- **A few ms ahead.** A line's frame goes into the stream 2-3 buffer times after it is queued (up to 3.2 ms with
  the default buffer). Its TIME is computed from its place in the stream: within ~1.5 µs of its last bit, as on the
  RMT. `start_delay: auto` adds that time, so group starts stay together, also with RMT lines.
- **Telemetry.** A poll frame's lane stays low for its response window while the other lanes keep sending. The
  line's `direction_pin` drops ~3-6 µs after the poll frame's last bit (2 µs after it, + the timer's 1 µs steps and
  its interrupt; modelled, to be confirmed on the bench): a timer alarm (GPTimer) set from the stream's timing, not
  the RMT's interrupt.
- **Firmware updates** run on a lane as on an RMT line: each page's pause is the lane's quiet after that frame.
- **When the CPU is late.** If the render task misses its buffer (more than ~1 ms late, e.g. a long flash write with
  the cache off), the DMA sends the last buffer again: frames on the wire at that moment are lost (the boards drop
  them, as a disturbed frame), and the stream's clock moves by the repeat, so frames queued before it go out again
  later with a new TIME. They count in the line's `rmt_underruns` ([Diagnostics](#diagnostics)) and the bus logs it.
  The RMT has the same weakness (its refill interrupts).

**Costs** (16 lanes, 20 MHz, 1 ms buffers):

| | |
|---|---|
| RAM | the two DMA buffers 2 x 41.5 KB (8 lanes: 2 x 20.7 KB; 10 MHz: 2 x 21.5 KB), 2.7 KB of render state, a 4 KB task stack, per line a copy of its frame (8 boards: 264 B, 50 boards: 1.6 KB); ~6 KB static (the bus and its drivers) |
| CPU | the render task: ~50,000 instructions per ms with all 16 lines sending back to back (~13 % of one of the P4's two cores at 400 MHz: counted from the RISC-V code, the render measured on a PC), ~1 % at rest (a refresh every 200 ms per line); no refill interrupts (the RMT: one every ~32 µs per busy line) |
| Flash | ~26 KB (11.6 KB of it ESP-IDF's PARLIO, GDMA and GPTimer drivers); a build without PARLIO grows 0.4 KB (the shared-UART turn-taking) |

### Several lines on one UART

A port answers on its own RS-485 receiver. Several ports can share one UART when each port's receiver can be
switched off: give each line a `receiver_enable_pin` (high = the receiver drives the UART's RX; `inverted: true` for
a /RE input). The hub turns it on for its own poll frame and off once its window is read, and the lines on one UART
take turns: **one window per UART at a time**. Lines on different UARTs still answer at the same time.

- **Hardware:** every port's receiver output (RO) on the one RX pin; RO must go high-impedance while its receiver is
  off (/RE high; most transceivers do, check the part). A 1-of-N decoder or an I/O expander can drive the enables:
  any output pin will do (it switches before the poll frame's last bit, not in an interrupt).
- **The configuration check** allows a shared UART only when every line on it has a `receiver_enable_pin`.
- **The cost:** a line's polls wait while another line of its UART has a window open (~5 ms each, about one a second
  per line): 4 lines on a UART use ~2 % of its time. Works for RMT lines and PARLIO lanes alike.

---

## 2. Ultra-Mode.md: "Several lines with telemetry"

Replace the first bullet:

> - **One UART per line.** The configuration check refuses two lines on one UART: both RS-485 receivers would drive
>   the same RX pin, and a line's windows are timed on that line only. The ports also need names (and, if given,
>   devices) of their own: their entities are named after them.

with:

> - **A UART per line, or one shared by several lines** that each have a `receiver_enable_pin` (see "Several lines
>   on one UART"): then they take turns, one window at a time. Without those pins the configuration check refuses a
>   shared UART (both receivers would drive the same RX pin). The ports also need names (and, if given, devices) of
>   their own: their entities are named after them.

and the last bullet ("More than 4 lines would need another transmitter than the RMT ... not built") with:

> - **More than 4 lines:** the ESP32-P4's PARLIO, up to 16 lines (see [PARLIO](#parlio-esp32-p4-up-to-16-lines)).

In the table's ESP32-P4 row: "Data lines: 4 (RMT), up to 16 more with PARLIO".

---

## 3. Development.md

**Layout** (the dig2analog_ultra row): add `ultra_parlio.h` (the PARLIO lanes: the bit grid, the stream's clock, a
lane's mailbox, the renderer; no ESP-IDF dependency), `parlio_timing.py` (the sample grid, for the configuration
check and the code generation); and a row:

| `components/dig2analog_ultra_parlio/` | [PARLIO](Ultra-Mode#parlio-esp32-p4-up-to-16-lines): the bus (the PARLIO TX unit, its two DMA buffers, the render task, the direction pins' timer) |

**Tests**, add:

```bash
python tests/run_ultra_parlio_test.py        # PARLIO lanes, shared UARTs; its negative controls and the render's cost
```

- **PARLIO test:** the bus modelled as `ultra_parlio_bus.cpp` drives it (chunk after chunk, the render at each
  buffer switch, the timer), the hub's own loop / transmit / window and update code cut from the sources:
  - every lane's waveform, sample for sample, is what the RMT encoder sends with the lane's timing on the grid (16 and
    8-bit samples, 20 and 10 MHz, inverted lanes, lanes with their own high times; both render paths); the lanes
    independent of each other; the rounding within the Physical Layer;
  - TIME: 1,000 frames of real schedulers (1 to 50 boards) on 16 lanes, each header's TIME within 1.5 µs of its last
    edge on the stream;
  - windows: a poll frame's lane low for its window while 15 lanes send; the direction pin 3-6 µs after the last bit,
    never before it;
  - slips: a render late by 1.3 buffers (a buffer played twice): foreseen, the clock moves, the frames queued before
    it refused, TIME right after it;
  - 16 lanes, 4 UARTs shared by 4 lanes each, with every frame decoded from the stream as a board sees it: windows of
    one UART never overlap (never two receivers on at once), windows of different UARTs do, every receiver answers
    with its own values, a silent lane leaves the others answering;
  - RMT lines sharing a UART (the same turn-taking);
  - an update session on a lane: every frame as the session built it, every page pause on the wire;
  - negative controls: the test against copies of the sources with a frame rendered a group late, the turn-taking
    gone, the direction pin let go of a byte early or 200 µs late: each must fail;
  - the render's cost per buffer (built with -O2).
- **Compile test:** `ultra-p4-parlio.yaml`: 16 lanes, telemetry on 4 UARTs shared by 4 lanes each (the receiver
  enables on an MCP23017), a light per lane, an RMT line next to them.

**Future options:** replace "more than 4 data lines (not built)" and "one UART for several ports (not built)" with:

> ## PARLIO and shared UARTs (built, not on the bench yet)
>
> Up to 16 lines on an ESP32-P4 (`dig2analog_ultra_parlio`) and several lines on one telemetry UART
> (`receiver_enable_pin`): see [PARLIO](Ultra-Mode#parlio-esp32-p4-up-to-16-lines). Still open:
> - the bench: the PARLIO stream on a scope (the timing, the direction pins' timer, a buffer switch), the render
>   task's real cost on a P4;
> - the ESP32-S31 (16-bit PARLIO in its preliminary datasheet) once ESP-IDF supports it;
> - the P4's BitScrambler could expand the bits into the waveform in hardware (a fraction of the CPU and of the
>   buffers);
> - the 16-port sender's select-line hardware (2 select lines shared by all groups + a decoder per group) enables
>   port k of every group at once: windows of different groups would then have to be on the same port, which the
>   per-line `receiver_enable_pin` doesn't express.

**Status**, add:

- **ESP32-P4, PARLIO:** compiles with 16 lanes and telemetry on 4 shared UARTs (`tests/build/ultra/ultra-p4-parlio.yaml`,
  in CI); host-tested; not on hardware yet.
