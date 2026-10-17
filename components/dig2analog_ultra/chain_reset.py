"""The reset gap a normal Ultra line needs when an Ultra-Digital receiver (ultra_digital, chain mode) is its first board.

The receiver classifies a frame (its slot and its header) while the next one may already be on the way: its main loop shares the 50 us
tick with the frame capture. Behind other boards the gap it sees is 320 us longer for each of them (each removes its slot from the front
of the frame); alone on its line it sees the hub's reset and nothing more. Its firmware takes a frame that starts 150 us after the last
one ended (test/test_ud_chain_budget.py in the firmware repository), so the default reset of 300 us has a margin of 150 us. A receiver
that is the first board gets at least CHAIN_FIRST_RESET_NS between the frames whatever reset_time says: a margin of 350 us, for a
burst of queued commands that is about 12 % fewer frames a second.

Pure Python (no ESPHome imports): tests/run_ultra_chain_reset_test.py loads this file on its own."""

CHAIN_FIRST_RESET_NS = 500000


def chain_first_receiver(hub_conf, lights) -> bool:
    """True if an ultra_digital light is board 0 of this hub, which is a normal Ultra line (no digital: and no PARLIO lane)"""
    if "digital" in hub_conf or "parlio_id" in hub_conf:
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
