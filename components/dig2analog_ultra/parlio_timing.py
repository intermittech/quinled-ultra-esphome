"""PARLIO lanes (dig2analog_ultra_parlio, ESP32-P4): the bus' sample grid, shared by the bus (its layout) and the hubs
on it (their timing on that grid, so their frames' TIME is the stream's). ultra_parlio.h renders it.

- A sample rate that divides the PARLIO clock (PLL_F160M) and has a whole number of ns per sample.
- Each lane's high times rounded to the nearest sample; every bit of the bus takes the same period (the longest bit
  of its lanes, rounded): a shorter bit's low time is stretched to it.
- The rounded times must stay inside the protocol's Physical Layer limits.
- A frame starts on a group (8 bits); the quiet after it (the reset) is rounded up to whole groups.
- The stream is cut into chunks of whole groups (two DMA buffers), each a multiple of 128 bytes (DMA / cache lines).
"""

PARLIO_CLOCK_HZ = 160_000_000
# Physical Layer (ns): '0' high, '1' high, low inside a frame; the reset gap
T0H_LIMITS = (220, 380)
T1H_LIMITS = (580, 2000)
TL_LIMITS = (450, 10000)
RESET_MIN_NS = 280_000
BUFFER_ALIGN = 128


def rate_ok(rate_hz: int) -> bool:
    return rate_hz > 0 and PARLIO_CLOCK_HZ % rate_hz == 0 and 1_000_000_000 % rate_hz == 0


def to_samples(ns: int, sample_ns: int) -> int:
    """Nearest sample, halves up."""
    return (ns + sample_ns // 2) // sample_ns


def lane_samples(timing: tuple, sample_ns: int) -> tuple:
    """(bit0_high, bit0_low, bit1_high, bit1_low) in ns -> the same in samples."""
    return tuple(to_samples(t, sample_ns) for t in timing)


def layout(rate_hz: int, lane_count: int, lanes: dict, buffer_ns: int) -> dict:
    """lanes: {lane: (bit0_high, bit0_low, bit1_high, bit1_low, reset) in ns}. The bus' layout and each lane's timing
    on it; 'errors': what breaks the Physical Layer (empty when it fits)."""
    sample_ns = 1_000_000_000 // rate_hz
    errors = []
    highs, period = {}, 0
    for lane, (b0h, b0l, b1h, b1l, _reset) in sorted(lanes.items()):
        h0, l0, h1, l1 = lane_samples((b0h, b0l, b1h, b1l), sample_ns)
        highs[lane] = (h0, h1)
        period = max(period, h0 + l0, h1 + l1)
        for what, ns, (lo, hi) in (("bit0_high", h0 * sample_ns, T0H_LIMITS), ("bit1_high", h1 * sample_ns, T1H_LIMITS)):
            if not lo <= ns <= hi:
                errors.append(
                    f"lane {lane}: {what} becomes {ns} ns at {rate_hz / 1e6:g} MHz ({sample_ns} ns per sample), outside "
                    f"the Physical Layer's {lo}-{hi} ns"
                )
        if h0 >= h1:
            errors.append(f"lane {lane}: bit0_high must stay shorter than bit1_high on the sample grid")
    width = 8 if lane_count <= 8 else 16
    out = {"sample_ns": sample_ns, "period": period, "width": width, "highs": highs, "errors": errors, "lanes": {}}
    if not lanes:
        return out
    for lane, (h0, h1) in highs.items():
        for what, low in (("bit0_low", period - h0), ("bit1_low", period - h1)):
            if not TL_LIMITS[0] <= low * sample_ns <= TL_LIMITS[1]:
                errors.append(f"lane {lane}: {what} becomes {low * sample_ns} ns, outside the Physical Layer's "
                              f"{TL_LIMITS[0]}-{TL_LIMITS[1]} ns")
    group_ns = 8 * period * sample_ns
    group_bytes = 8 * period * width // 8
    groups = max(1, -(-buffer_ns // group_ns))
    while (groups * group_bytes) % BUFFER_ALIGN:
        groups += 1
    out.update(group_ns=group_ns, chunk_groups=groups, chunk_bytes=groups * group_bytes, chunk_ns=groups * group_ns)
    for lane, (h0, h1) in highs.items():
        reset = lanes[lane][4]
        reset_groups = -(-reset // group_ns)
        # the hub's timing on the grid: its frames' TIME and length are the stream's
        out["lanes"][lane] = (h0 * sample_ns, (period - h0) * sample_ns, h1 * sample_ns, (period - h1) * sample_ns,
                              reset_groups * group_ns)
    return out
