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
    # buffer_time: 1ms              # (auto: 1 ms, longer where the DMA needs it)
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
| `buffer_time` | auto | one of the two DMA buffers (0.25-10 ms): a frame starts up to ~3 of them after it was queued; a longer one rides out a busier CPU, a shorter one saves RAM. Auto: 1 ms, longer where the DMA's look-ahead needs it (8 pins at 10 MHz: 1.34 ms); the configuration check refuses a shorter one |
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
  RMT. `start_delay: auto` adds that time, and counts every command of a group change at the lane's pace (a lane
  takes a frame every two buffers at most: long buffers with short frames), so group starts stay together, also
  with RMT lines.
- **Telemetry.** A poll frame's lane stays low for its response window while the other lanes keep sending. The
  line's `direction_pin` drops ~3-6 µs after the poll frame's last bit (2 µs after it, + the timer's 1 µs steps and
  its interrupt; modelled, to be confirmed on the bench): a timer alarm (GPTimer) set from the stream's timing, not
  the RMT's interrupt.
- **Firmware updates** run on a lane as on an RMT line: each page's pause is the lane's quiet after that frame.
- **When the CPU is late.** The DMA would play the last buffer again if the next one isn't there in time, and a
  buffer played twice puts whole, valid slots in the wrong boards' places. So it never gets that far: each buffer
  must be handed over a margin before the one playing ends (the DMA reads two descriptors ahead: 0.25 ms at 20 MHz
  with 16 pins), and a timer checks it. A buffer that misses it (the render task stalled, e.g. a long flash write
  with the cache off) stops the stream there: every pin goes to its idle level at once, so the frames on the wire are
  cut short and the boards drop them, as a disturbed frame. The render task then restarts the stream (idle buffers
  first) and the lines send again, their frames with new starts and TIMEs; nothing queued before is sent from the
  old timeline. Lost frames count in the line's `rmt_underruns` ([Diagnostics](#diagnostics)), the bus logs the
  restarts. The timer's interrupt and what it calls are in IRAM and run with the cache off (ESP-IDF's
  `CONFIG_GPTIMER_ISR_CACHE_SAFE`, set by the component), so this check holds during flash writes too.
- **The render task** runs on core 1 at priority 20. Core 1 is ESPHome's loop core (priority 1): the render takes
  ~0.1 ms of every buffer from it, while core 0 keeps Wi-Fi / Ethernet, lwIP, esp_timer and IPC at their own (higher)
  priorities, untouched. Being late only costs a restart (above), never a wrong frame.

**Costs** (16 lanes, 20 MHz, 1 ms buffers):

| | |
|---|---|
| RAM | the two DMA buffers 2 x 41.5 KB (8 lanes: 2 x 20.7 KB; 10 MHz: 2 x 21.5 KB), 2.7 KB of render state, a 4 KB task stack, per line a copy of its frame (8 boards: 264 B, 50 boards: 1.6 KB); ~8 KB static (the bus, its drivers, the timer's interrupt path in IRAM) |
| CPU | the render task: ~50,000 instructions per ms with all 16 lines sending back to back (~13 % of one of the P4's two cores at 400 MHz: counted from the RISC-V code, the render measured on a PC), ~1 % at rest (a refresh every 200 ms per line); no refill interrupts (the RMT: one every ~32 µs per busy line) |
| Flash | ~28 KB (~12 KB of it ESP-IDF's PARLIO, GDMA and GPTimer drivers); a build without PARLIO grows 0.5 KB (the shared-UART turns) |

### Several lines on one UART

A port answers on its own RS-485 receiver. Several ports can share one UART when each port's receiver can be
switched off: give each line a `receiver_enable_pin` (high = the receiver drives the UART's RX; `inverted: true` for
a /RE input). The hub turns it on for its own poll frame and off once its window is read, and the lines on one UART
take turns: **one window per UART at a time**. Lines on different UARTs still answer at the same time.

- **Hardware:** every port's receiver output (RO) on the one RX pin; RO must go high-impedance while its receiver is
  off (/RE high; most transceivers do, check the part). A 1-of-N decoder or an I/O expander can drive the enables:
  any output pin will do (it switches before the poll frame's last bit, not in an interrupt).
- **The configuration check** allows a shared UART only when every line on it has a `receiver_enable_pin`.
- **Turns:** when a window ends, the line that has waited longest goes next, so a busy line (a long chain with short
  intervals) can't keep the UART from the others; each line waits at most about one window of every other line.
  Works for RMT lines and PARLIO lanes alike.
- **The capacity** is shared: a window takes ~5 ms on a short chain at 1 Mbaud (up to ~200 ms on a long one at 250
  kbaud with slow bits), and the lines on one UART get their polls in turn. With the default intervals 4 short lines
  use ~2 % of a UART's time; lines that need more (many receivers, short intervals) get proportionally fewer windows
  each, so give them a UART of their own.

#### Later: shared select lines

The 16-port sender's design selects a receiver with 2 select lines shared by all 4 groups and a 1-of-4 decoder per
group: setting the select lines to k enables port k of **every** group at once. The per-line `receiver_enable_pin` can
express that later without changing a line's configuration much: the select lines become one `receiver_select`
(the pins, given once), and a line names its index on it instead of its own enable pin. The turn-taking then has a
second rule next to "one window per UART": while a window is open the select lines hold its index, so lines of other
groups may only open windows on the same index (port k of every group answering at once is fine: different UARTs),
and a line with another index waits. The ordering (longest waiting first) stays; lines with the same index are
served together, which keeps the 4 groups answering in parallel.

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
  - a late render (by less than its margin: nothing happens; by more: the timer stops the stream before any buffer
    plays twice): every frame on the wire with a valid header is exactly a frame a hub posted, once, with its real
    TIME; the stream restarts and frames flow (also 8-board frames across chunk boundaries);
  - a late switch interrupt changes nothing (no time is taken from an interrupt); a stop cancels the direction pins'
    releases of the frames it cut, never moves one;
  - 16 lanes, 4 UARTs shared by 4 lanes each, with every frame decoded from the stream as a board sees it (also
    through stops of the stream): windows of one UART never overlap (never two receivers on at once), windows of
    different UARTs do, every receiver answers with its own values, a silent lane leaves the others answering;
  - RMT lines sharing a UART (the same turn-taking); a busy 64-receiver line at 250 kbaud can't starve the other line
    on its UART; a poll frame that failed to go out doesn't keep the UART through an update;
  - a group change of 5 lights on an RMT line and a lane with 10 ms buffers: every first copy before its start;
  - an update session on a lane: every frame as the session built it, every page pause on the wire; reservations
    that expired and frames the stream refused go again; a stalled stream ends the session in bounded time (also
    with an abort);
  - the feature check: dig2analog_ultra.cpp / .h preprocessed with every combination of telemetry, PARLIO and
    updates, every member the .cpp names declared in the .h;
  - negative controls: the checks against copies of the sources with one thing broken (no render deadline, a stop
    that leaves the releases of cut frames armed, the release callback named without telemetry, the update's
    refusals / expired reservations / abort ignored, the group start without a lane's cadence, the UART to whichever
    line asks first, an update keeping a dropped poll's turn, no turn-taking, a frame rendered a group late, the
    direction pin let go of a byte early or 200 µs late): each must fail;
  - the render's cost per buffer (built with -O2).
- **Compile tests:** `ultra-p4-parlio.yaml`: 16 lanes, telemetry on 4 UARTs shared by 4 lanes each (the receiver
  enables on an MCP23017), a light per lane, an RMT line next to them; `ultra-p4-parlio-lights.yaml`: 8 lanes at 10
  MHz, lights only (no telemetry).

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
> - `CONFIG_SPI_FLASH_AUTO_SUSPEND` (flash writes that let code from flash run on) would let the render task go on
>   through flash writes instead of the stream stopping; ESP-IDF supports it on specific flash chips only (and an
>   OTA to it needs a bootloader with it), so it isn't set;
> - the 16-port sender's select-line hardware (2 select lines shared by all groups + a decoder per group) enables
>   port k of every group at once: see "Later: shared select lines".

**Status**, add:

- **ESP32-P4, PARLIO:** compiles with 16 lanes and telemetry on 4 shared UARTs (`tests/build/ultra/ultra-p4-parlio.yaml`,
  in CI); host-tested; not on hardware yet.
