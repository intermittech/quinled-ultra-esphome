The three `*_r10` files freeze the ledger from commit
`a73ff9c59e44cb76f21343e2877133ceb875f9b5`, before the round-11 changes.

`ultra_ledger_r10.h` changes only its namespace, its two relative include paths,
and qualifies two `receiver_counters` calls to avoid argument-dependent lookup
into the current component namespace. The two `.inc` files contain the original
hub ledger service/save and line-accounting method definitions.

`run_ultra_ledger_test.py` extracts the current hub definitions from production.
The differential suite executes both versions against the same preference and
line fixtures. The preference model follows the installed ESPHome ESP32 flash
backend: key-based, variable lengths, an owned pending copy, exact-length loads,
sync read-back at matching lengths, and a separate size-key commit.

The tests cover arithmetic equivalence, old/new LDG3 reading in both directions,
power-loss boundaries, failed saves/syncs, every truncation, CRC corruption,
historical entries, allocation changes, generation wrap, and 452 days of lit-time
accounting. Transition detection remains per loop; only repeated clock accounting
is gated to 50 ms. Reads and saves flush pending time to each line's last observed
timestamp. The blind-gate negative control demonstrates why sampling transitions
every 50 ms cannot preserve the old behavior.

The optional `ultra_ledger_probe.cpp` measures Xtensa ESP32 object layouts and the
`add_time` wrapper code. Its libstdc++ control-block probe is target-toolchain
specific. Host timings are printed by the differential suite and are not MCU
latency measurements.
