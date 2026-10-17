"""The reset gap a normal Ultra line needs when an Ultra-Digital receiver (ultra_digital, chain mode) is its first board.

The receiver classifies a frame (its slot and its header) while the next one may already be on the way: its main loop shares the 50 us
tick with the frame capture. Behind other boards the gap it sees is 320 us longer for each of them (each removes its slot from the front
of the frame); alone on its line it sees the hub's reset and nothing more. The firmware frees its capture 126 .. 189 us after a frame's
last edge while a light effect runs (a 40-pixel Rainbow: it takes a frame that starts 150 us after the last one ended,
test/test_ud_chain_budget.py in the firmware repository). A heavy effect (Pacifica on 300 or 1,080 pixels renders about 27 k cycles
between two looks at the line) holds the capture longer and drops frames at some phases of the 50 us tick when the reset is 300 us; at
500 us it does not (ud-fx-verify, round 2, N1). A receiver that is the first board therefore gets at least CHAIN_FIRST_RESET_NS between
the frames whatever reset_time says: for a burst of queued commands that is about 12 % fewer frames a second.

The floor is applied to the hub's validated reset_time (final validation), so an RMT line and a PARLIO lane (whose bus layout reads the
hubs' reset_time) both get it.

Pure Python (no ESPHome imports): tests/run_ultra_chain_reset_test.py loads this file on its own."""

CHAIN_FIRST_RESET_NS = 500000


def chain_first_receiver(hub_conf, lights) -> bool:
    """True if an ultra_digital light is board 0 of this hub, which is a normal Ultra line (no digital:; an RMT line or a PARLIO lane)"""
    if "digital" in hub_conf:
        return False
    hub_id = str(hub_conf["id"])
    return any(
        light.get("platform") == "ultra_digital"
        and str(light.get("dig2analog_ultra_id")) == hub_id
        and light.get("board") == 0
        for light in lights or []
    )


def reset_with_floor_ns(reset_ns: int, hub_conf, lights) -> int:
    """the hub's reset_time in ns, raised to CHAIN_FIRST_RESET_NS when a receiver is the first board of the line"""
    if reset_ns < CHAIN_FIRST_RESET_NS and chain_first_receiver(hub_conf, lights):
        return CHAIN_FIRST_RESET_NS
    return reset_ns
