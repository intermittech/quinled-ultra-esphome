# Tests

```bash
python tests/run_all.py        # every host step of CI's host job (read from .github/workflows/ci.yml), one at a time
python tests/check_pins.py     # the pins only
```

Run them with ESPHome's Python (`pip install esphome==2026.9.1 ziglang==0.16.0`).

**Required checkouts.** The host tests run against the protocol and the receiver firmware at the commits `ci.yml`
pins (`PROTOCOL_REF`, `FIRMWARE_REF`): `../quinled-ultra-protocol` and `../quinled-ultra-firmware`, or
`ULTRA_PROTOCOL=<checkout>` / `ULTRA_FIRMWARE=<checkout>`. A missing or mismatched one fails; nothing is skipped.

**Pins.** `check_pins.py` fails when one moves alone: the protocol checkout at `PROTOCOL_REF`, the firmware checkout
at `FIRMWARE_REF`, the firmware's own protocol pin (`test/test_protocol_copy.py`) = `PROTOCOL_REF`, one
`ultra_protocol.h` in three copies, the vendored sender sources = the protocol's `sender/src`. Moving the protocol:
protocol commit, then the firmware's pin and header copy, then `PROTOCOL_REF`, `FIRMWARE_REF` and the copies here.

**Receiver contracts** (always on): the board engine (host, stream), the wire-3 contract, the telemetry answer path
and the digital port replayed into the receiver's delay line, all on the pinned firmware's sources. The firmware's
own gate (`test/run_cross_repo.py` there) runs the same contracts against its working tree.

`ULTRA_TEST_JOBS` (default 4) caps the parallel compiles inside a step. Build output goes to `tests/build/`.
Details: the wiki's [Development](https://github.com/intermittech/quinled-ultra-esphome/wiki/Development) page.
