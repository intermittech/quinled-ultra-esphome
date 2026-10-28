"""Ultra protocol 0.3 processing gap, applied to every normal Ultra line."""
CHAIN_FIRST_RESET_NS = 500000
def chain_first_receiver(hub_conf, lights):
    return "digital" not in hub_conf
def reset_with_floor_ns(reset_ns, hub_conf, lights):
    return max(reset_ns, CHAIN_FIRST_RESET_NS)
