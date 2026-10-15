"""The line's `telemetry:` (QuinLED Diff-Power ultra receivers report back): its schema, the entities of each receiver
(a Home Assistant device of its own, `device_id`), the port's summary, and the lights' estimates. Used by __init__.py
and light.py; the logic is ultra_telemetry.h, the entities ultra_receiver.h."""

from esphome import pins
import esphome.codegen as cg
from esphome.components import binary_sensor, sensor, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_DEVICE_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ID,
    CONF_NAME,
    CONF_RX_BUFFER_SIZE,
    CONF_RX_PIN,
    CONF_UART_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_PERCENT,
    UNIT_SECOND,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

from esphome.core import CORE, ID

ns = cg.esphome_ns.namespace("dig2analog_ultra")
UltraReceiver = ns.class_("UltraReceiver", cg.Component)
UltraTelemetryPort = ns.class_("UltraTelemetryPort", cg.Component)
UltraLightTelemetry = ns.class_("UltraLightTelemetry")   # not a Component: made at codegen, attach()ed

CONF_TELEMETRY = "telemetry"
CONF_RECEIVERS = "receivers"
CONF_DIRECTION_PIN = "direction_pin"
CONF_POLL_INTERVALS = "poll_intervals"
CONF_DIAGNOSTICS = "diagnostics"
CONF_LIGHT_ENTITIES = "light_entities"
CONF_PORT_ID = "port_id"
CONF_PORT_LIMIT = "port_limit"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_LOAD_WARNING = "load_warning"
CONF_LOAD_PROBLEM = "load_problem"
CONF_LOAD_PROBLEM_TIME = "load_problem_time"
POLL_PAGES = ["status", "power", "energy", "supply", "health", "lifetime", "diag"]
# the UART's baud rate is the rate the receivers answer at (ULTRA_RW_RATE_*)
RW_RATES = {1000000: 0, 500000: 1, 250000: 2}
RW_RX_BUFFER_MIN = 1024
# none: Power, Energy, Status (+ its detail), Problem; basic: + the shown diagnostics; full (default): + the hidden ones
# (disabled in Home Assistant until enabled; ~0.8 KB of the ESP32's RAM per receiver)
DIAG_LEVELS = ["none", "basic", "full"]


def _value(unit, decimals, device_class=None, state_class=STATE_CLASS_MEASUREMENT, icon=None, diagnostic=True):
    kw = {"unit_of_measurement": unit, "accuracy_decimals": decimals, "state_class": state_class}
    if device_class:
        kw["device_class"] = device_class
    if icon:
        kw["icon"] = icon
    if diagnostic:
        kw["entity_category"] = ENTITY_CATEGORY_DIAGNOSTIC
    return sensor.sensor_schema(**kw)


def _count(icon):
    return sensor.sensor_schema(
        icon=icon, accuracy_decimals=0, state_class=STATE_CLASS_TOTAL_INCREASING, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    )


def _text(icon, diagnostic=True):
    if diagnostic:
        return text_sensor.text_sensor_schema(icon=icon, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
    return text_sensor.text_sensor_schema(icon=icon)


# A receiver's entities: key -> (kind, index in its C++ list, level, name, schema). Level 0: shown on its device (the
# few that matter every day); 1: diagnostics, shown; 2: diagnostics, disabled in Home Assistant until enabled.
RECEIVER_ENTITIES = {
    # what it draws now, measured (W); its lifetime energy (Wh, counting up: the energy dashboard)
    "power": ("sensor", 0, 0, "Power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "energy": (
        "sensor", 1, 0, "Energy",
        _value(UNIT_WATT_HOURS, 0, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING, diagnostic=False),
    ),
    # No answer, Tripped, Channel fault, Too hot, Signal loss, Supply dips, Calibration suggested (these are problems);
    # Limiting, Calibrating, Soft start, Held dark, Blinking, OK
    "status": ("text", 0, 0, "Status", _text("mdi:list-status", diagnostic=False)),
    # on when the Status needs attention (the problems above)
    "problem": ("binary", 0, 0, "Problem", binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM)),
    # the Status in one plain line ("No answer. Receivers 1 and 2 before it answer, so check its cable or power."):
    # shown with it (ESPHome entities have no attributes)
    "status_detail": ("text", 1, 0, "Status detail", _text("mdi:text-box-outline", diagnostic=False)),
    "bus_voltage": ("sensor", 2, 1, "Bus voltage", _value(UNIT_VOLT, 2, DEVICE_CLASS_VOLTAGE)),
    # the MCU's own sensor: a trend and over-temperature value (its offset varies per chip by up to ~20 C)
    "mcu_temperature": ("sensor", 3, 1, "MCU temperature", _value(UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE)),
    "firmware": ("text", 2, 1, "Firmware", _text("mdi:chip")),
    "input_current": ("sensor", 4, 2, "Input current", _value(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT)),
    "uptime": (
        "sensor", 5, 2, "Uptime",
        _value(UNIT_SECOND, 0, DEVICE_CLASS_DURATION, STATE_CLASS_TOTAL_INCREASING, "mdi:timer-outline"),
    ),
    "last_reset": ("text", 3, 2, "Last reset", _text("mdi:restart")),
    "last_trip": ("text", 4, 2, "Last trip", _text("mdi:flash-alert-outline")),
    "last_dip": ("text", 5, 2, "Last dip", _text("mdi:transmission-tower-off")),
    # its lifetime counts (kept across power cuts; protocol 0.2.5, receiver firmware 0.6.9), before that since its
    # power-up
    "trips": ("sensor", 6, 2, "Trips", _count("mdi:flash-off")),
    "supply_dips": ("sensor", 7, 2, "Supply dips", _count("mdi:transmission-tower-off")),
    # since its power-up
    "limit_events": ("sensor", 8, 2, "Limit events", _count("mdi:speedometer-slow")),
    # the hub's own statistic: polls without an answer, % since the ESP started
    "missed_polls": ("sensor", 9, 2, "Missed polls", _value(UNIT_PERCENT, 1, icon="mdi:access-point-network-off")),
    # its position on the port and its chip's UID16 ("2 · UID 9C02")
    "link": ("text", 6, 2, "Position and UID", _text("mdi:identifier")),
    "channel_health": ("text", 7, 2, "Channel health", _text("mdi:heart-pulse")),
    # along the port's cable: the sender's voltage minus this one's; the segment before it (smoothed, from 0.3 A on)
    "voltage_drop": ("sensor", 10, 2, "Voltage drop", _value(UNIT_VOLT, 2, DEVICE_CLASS_VOLTAGE)),
    "cable_resistance": ("sensor", 11, 2, "Cable resistance", _value("m\u03a9", 0, icon="mdi:omega")),
    # protocol 0.2.5 (receiver firmware 0.6.9): the LED supply after its buck; its highest input current (~100 ms)
    # between two reads (about a minute); the supply / cable impedance it measured; its odometer
    "led_supply": ("sensor", 12, 2, "LED supply", _value(UNIT_VOLT, 2, DEVICE_CLASS_VOLTAGE)),
    "input_current_peak": ("sensor", 13, 2, "Input current peak", _value(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT)),
    "supply_impedance": ("sensor", 14, 2, "Supply impedance", _value("m\u03a9", 0, icon="mdi:omega")),
    "lit_hours": (
        "sensor", 15, 2, "Lit hours",
        _value(UNIT_HOUR, 1, DEVICE_CLASS_DURATION, STATE_CLASS_TOTAL_INCREASING, "mdi:lightbulb-on-outline"),
    ),
    "power_ups": ("sensor", 16, 2, "Power-ups", _count("mdi:power")),
}

# The port's summary (on the hub's device, or the telemetry's own device_id): key -> (kind, index, name, schema)
PORT_ENTITIES = {
    # at the sender: its voltage x the port current (without a supply_voltage: learned, else the receivers' power)
    "port_power": ("sensor", 0, "power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    # the receivers' input currents together, and against the port's limit (3 x 750 mA PTCs = 2.25 A)
    "port_current": ("sensor", 1, "current", _value(UNIT_AMPERE, 2, DEVICE_CLASS_CURRENT, diagnostic=False)),
    "port_load": ("sensor", 2, "load", _value(UNIT_PERCENT, 0, icon="mdi:gauge", diagnostic=False)),
    # the cable along the chain: W, and in % of the port's power
    "port_cable_loss": ("sensor", 3, "cable loss", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER)),
    "port_cable_loss_share": ("sensor", 4, "cable loss share", _value(UNIT_PERCENT, 1, icon="mdi:cable-data")),
    # (0.2.5) the receivers' highest input currents between two reads (about a minute), added up, against the limit
    "port_peak_load": ("sensor", 5, "peak load", _value(UNIT_PERCENT, 0, icon="mdi:gauge-full")),
    # on when a receiver has a problem, or the load is at 100 %, or at load_problem for load_problem_time
    "port_problem": ("binary", 0, "problem", binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM)),
    "port_receivers": ("text", 0, "receivers", _text("mdi:access-point-network", diagnostic=False)),
    # "OK", "91 % load", "No answer", a receiver's problem, "Peaks 104 %", "2 limiting"; and the plain line under it
    "port_status": ("text", 1, "status", _text("mdi:list-status", diagnostic=False)),
    "port_status_detail": ("text", 2, "status detail", _text("mdi:text-box-outline", diagnostic=False)),
}

# The estimates of a light on a receiver with several lights (on the light's own device): key -> (kind, name, schema)
LIGHT_ENTITIES = {
    "estimated_power": ("sensor", "Estimated power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "estimated_energy": (
        "sensor", "Estimated energy",
        _value(UNIT_WATT_HOURS, 1, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING, diagnostic=False),
    ),
    "status": ("text", "Status", _text("mdi:list-status", diagnostic=False)),
}


def _poll_interval(key, default, low, high):
    return cv.Optional(key, default=default), cv.All(
        cv.positive_time_period_milliseconds, cv.Range(min=low, max=high)
    )


RECEIVER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraReceiver),
        # the receiver's Home Assistant device (esphome: devices:), e.g. "Port 4 · Receiver 2" in its area
        cv.Optional(CONF_DEVICE_ID): cv.sub_device_id,
        **{cv.Optional(key): spec[4] for key, spec in RECEIVER_ENTITIES.items()},
    }
)

TELEMETRY_SCHEMA = cv.Schema(
    {
        # the UART with the hub's RS-485 receiver output on its rx_pin; its baud_rate is the receivers' answer rate:
        # 1000000 (default), 500000 (segments over ~50 m) or 250000 (beyond ~100 m); 8N1, rx_buffer_size >= 1024
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        # the port's name: its summary entities are "<name> power", "<name> problem", "<name> receivers"
        cv.Required(CONF_NAME): cv.string_strict,
        # the device of the port's summary (default: the hub's own device)
        cv.Optional(CONF_DEVICE_ID): cv.sub_device_id,
        # the receivers, from the start of the chain (a count, or one entry per receiver with its device_id). A
        # dig2analog board doesn't pass answers on, so receivers behind one can't be heard.
        cv.Required(CONF_RECEIVERS): cv.All(cv.ensure_list(RECEIVER_SCHEMA), cv.Length(min=1, max=64)),
        cv.Optional(CONF_DIAGNOSTICS, default="full"): cv.one_of(*DIAG_LEVELS, lower=True),
        # the sender port's current limit (3 x 750 mA PTCs), the load's warning and problem
        cv.Optional(CONF_PORT_LIMIT, default="2.25A"): cv.All(cv.current, cv.Range(min=0.1, max=20.0)),
        cv.Optional(CONF_LOAD_WARNING, default="90%"): cv.percentage,
        cv.Optional(CONF_LOAD_PROBLEM, default="95%"): cv.percentage,
        cv.Optional(CONF_LOAD_PROBLEM_TIME, default="30s"): cv.positive_time_period_milliseconds,
        # the sender's voltage (power at the sender, the first cable's loss); without it: receiver 1's highest
        cv.Optional(CONF_SUPPLY_VOLTAGE): cv.All(cv.voltage, cv.Range(min=5.0, max=60.0)),
        # estimated power / energy and a status per light, for the lights of a receiver with several lights
        cv.Optional(CONF_LIGHT_ENTITIES, default=True): cv.boolean,
        # the hub's RS-485 driver enable (high = the hub drives the line): let go of right after each poll frame, so
        # the receivers can answer. Leave it out when the hardware lets go of the line by itself.
        cv.Optional(CONF_DIRECTION_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_POLL_INTERVALS, default={}): cv.Schema(
            dict(
                [
                    # STATUS: voltage, current, temperature, the power state
                    _poll_interval("status", "3s", cv.TimePeriod(seconds=1), cv.TimePeriod(seconds=60)),
                    # POWER: limit, trips, power events; OUTPUTS (receivers with several lights): their currents
                    _poll_interval("power", "5s", cv.TimePeriod(seconds=1), cv.TimePeriod(seconds=60)),
                    _poll_interval("energy", "60s", cv.TimePeriod(seconds=10), cv.TimePeriod(hours=1)),
                    _poll_interval("supply", "60s", cv.TimePeriod(seconds=10), cv.TimePeriod(hours=1)),
                    _poll_interval("health", "60s", cv.TimePeriod(seconds=10), cv.TimePeriod(hours=1)),
                    # (0.2.5) LIFETIME: the odometer; DIAG: the running board, the fault memory (and after a restart)
                    _poll_interval("lifetime", "5min", cv.TimePeriod(seconds=30), cv.TimePeriod(hours=24)),
                    _poll_interval("diag", "5min", cv.TimePeriod(seconds=30), cv.TimePeriod(hours=24)),
                ]
            )
        ),
        cv.GenerateID(CONF_PORT_ID): cv.declare_id(UltraTelemetryPort),
        **{cv.Optional(key): spec[3] for key, spec in PORT_ENTITIES.items()},
    }
)


def _entity(value, name, device, disabled):
    ent = dict(value or {})
    ent.setdefault(CONF_NAME, name)
    if device is not None:
        ent.setdefault(CONF_DEVICE_ID, device)
    if disabled:
        ent.setdefault(CONF_DISABLED_BY_DEFAULT, True)
    return ent


def telemetry_defaults(config):
    """Before the hub's schema: the receivers as a list (a count is that many), and every entity they and the port
    have, named for their device (with a device_id: "Power" on the receiver's device; without: "<port> R2 power" on
    the hub's). Their device_id must be in each entity when it is validated: names are unique per device."""
    if not isinstance(config, dict) or not config.get(CONF_TELEMETRY):
        return config
    config = dict(config)
    tel = dict(config[CONF_TELEMETRY])
    level = str(tel.get(CONF_DIAGNOSTICS, "full")).lower()
    lvl = DIAG_LEVELS.index(level) if level in DIAG_LEVELS else 1
    port = str(tel.get(CONF_NAME, "Port"))
    tel[CONF_NAME] = port
    recs = tel.get(CONF_RECEIVERS)
    if recs is None:
        try:
            recs = int(config.get("num_boards", 1))
        except (TypeError, ValueError):
            recs = 1
    if isinstance(recs, int) and not isinstance(recs, bool):
        if not 1 <= recs <= 64:
            raise cv.Invalid("telemetry: receivers must be 1-64 (a count), or a list of receivers")
        recs = [{} for _ in range(recs)]
    if not isinstance(recs, list):
        raise cv.Invalid("telemetry: receivers is a count or a list (one entry per receiver, with its device_id)")
    out = []
    for k, item in enumerate(recs):
        if item is None:
            item = {}
        elif isinstance(item, str):
            item = {CONF_DEVICE_ID: item}
        if not isinstance(item, dict):
            raise cv.Invalid(f"telemetry: receiver {k + 1}: a device_id, or a mapping with device_id and entities")
        item = dict(item)
        device = item.get(CONF_DEVICE_ID)
        for key, (_kind, _index, ent_level, name, _schema) in RECEIVER_ENTITIES.items():
            if key not in item and ent_level > lvl:
                continue
            label = name if device is not None else f"{port} R{k + 1} {name[0].lower()}{name[1:]}"
            item[key] = _entity(item.get(key), label, device, ent_level == 2)
        out.append(item)
    tel[CONF_RECEIVERS] = out
    device = tel.get(CONF_DEVICE_ID)
    for key, (_kind, _index, name, _schema) in PORT_ENTITIES.items():
        label = f"{name[0].upper()}{name[1:]}" if device is not None else f"{port} {name}"
        tel[key] = _entity(tel.get(key), label, device, False)
    config[CONF_TELEMETRY] = tel
    return config


def uart_of(full, uart_id):
    for u in full.get("uart", []) or []:
        if u[CONF_ID] == uart_id:
            return u
    return None


def validate_telemetry_lines(full):
    """Across the lines: each line's telemetry has a UART of its own (its own RS-485 receiver: a line's response
    windows are timed on that line only). Two ports with one name or one device: ESPHome's own check refuses their
    entities' names (unique per device)."""
    hubs = full.get("dig2analog_ultra", []) or []
    uarts = {}
    for conf in hubs if isinstance(hubs, list) else [hubs]:
        tel = conf.get(CONF_TELEMETRY)
        if tel is None:
            continue
        hub = str(conf[CONF_ID])
        uart_id = str(tel[CONF_UART_ID])
        if uart_id in uarts:
            raise cv.Invalid(
                f"the telemetry of '{uarts[uart_id]}' and of '{hub}' use the same uart '{uart_id}': each line needs a "
                "UART of its own, with its own RS-485 receiver (a line's response windows are timed on that line only)"
            )
        uarts[uart_id] = hub


def validate_telemetry(full, conf, num_boards):
    """Final validation: the receivers fit the line, the UART is one the receivers can answer on."""
    tel = conf.get(CONF_TELEMETRY)
    if tel is None:
        return
    if len(tel[CONF_RECEIVERS]) > num_boards:
        raise cv.Invalid(
            f"telemetry: {len(tel[CONF_RECEIVERS])} receivers can't be more than num_boards ({num_boards})"
        )
    u = uart_of(full, tel[CONF_UART_ID])
    if u is None:
        return
    name = f"telemetry of '{conf[CONF_ID]}': uart '{tel[CONF_UART_ID]}'"
    if CONF_RX_PIN not in u:
        raise cv.Invalid(f"{name} needs an rx_pin (the RS-485 receiver's output)")
    if u[CONF_BAUD_RATE] not in RW_RATES:
        raise cv.Invalid(
            f"{name}: baud_rate {u[CONF_BAUD_RATE]} isn't a rate the receivers answer at; use 1000000, 500000 "
            "(segments over ~50 m) or 250000 (beyond ~100 m)"
        )
    if u.get("data_bits", 8) != 8 or str(u.get("parity", "NONE")) != "NONE" or u.get("stop_bits", 1) != 1:
        raise cv.Invalid(f"{name}: the answers are 8N1 (data_bits 8, parity NONE, stop_bits 1)")
    if u[CONF_RX_BUFFER_SIZE] < RW_RX_BUFFER_MIN:
        raise cv.Invalid(
            f"{name}: rx_buffer_size must be at least {RW_RX_BUFFER_MIN} (a window's answers, and the hub's own "
            "frames when its receiver hears them)"
        )


def lights_per_board(full, hub_id):
    counts = {}
    for light in full.get("light", []) or []:
        if light.get("platform") == "dig2analog_ultra" and light.get("dig2analog_ultra_id") == hub_id:
            counts[light["board"]] = counts.get(light["board"], 0) + 1
    return counts


async def _new(kind, conf):
    if kind == "sensor":
        return await sensor.new_sensor(conf)
    if kind == "text":
        return await text_sensor.new_text_sensor(conf)
    return await binary_sensor.new_binary_sensor(conf)


async def telemetry_to_code(hub, config):
    """The hub's telemetry: the poller, the receivers' entities, the port's summary."""
    tel = config[CONF_TELEMETRY]
    cg.add_define("USE_DIG2ANALOG_ULTRA_TELEMETRY")
    iv = tel[CONF_POLL_INTERVALS]
    receivers = tel[CONF_RECEIVERS]
    cg.add(
        hub.set_telemetry(
            await cg.get_variable(tel[CONF_UART_ID]),
            len(receivers),
            RW_RATES[uart_of(CORE.config, tel[CONF_UART_ID])[CONF_BAUD_RATE]],
            *[int(iv[k].total_milliseconds) for k in POLL_PAGES],
        )
    )
    if CONF_DIRECTION_PIN in tel:
        cg.add(hub.set_telemetry_direction_pin(await cg.gpio_pin_expression(tel[CONF_DIRECTION_PIN])))
    cg.add(
        hub.set_telemetry_port(
            tel[CONF_PORT_LIMIT],
            tel[CONF_SUPPLY_VOLTAGE] if CONF_SUPPLY_VOLTAGE in tel else cg.RawExpression("NAN"),
            tel[CONF_LOAD_WARNING] * 100.0,
            tel[CONF_LOAD_PROBLEM] * 100.0,
            int(tel[CONF_LOAD_PROBLEM_TIME].total_milliseconds),
        )
    )
    counts = lights_per_board(CORE.config, config[CONF_ID])
    for k, rec in enumerate(receivers):
        if counts.get(k, 0) > 1:
            cg.add(hub.set_telemetry_outputs(k, True))   # several lights: their currents (OUTPUTS)
        var = cg.new_Pvariable(rec[CONF_ID])
        await cg.register_component(var, rec)
        cg.add(var.set_hub(hub))
        cg.add(var.set_board(k))
        for key, (kind, index, _level, _name, _schema) in RECEIVER_ENTITIES.items():
            if key not in rec:
                continue
            ent = await _new(kind, rec[key])
            if kind == "sensor":
                cg.add(var.set_value(index, ent))
            elif kind == "text":
                cg.add(var.set_text(index, ent))
            else:
                cg.add(var.set_problem(ent))
    port = cg.new_Pvariable(tel[CONF_PORT_ID])
    await cg.register_component(port, {})
    cg.add(port.set_hub(hub))
    for key, (kind, index, _name, _schema) in PORT_ENTITIES.items():
        if key not in tel:
            continue
        ent = await _new(kind, tel[key])
        if kind == "sensor":
            cg.add(port.set_value(index, ent))
        elif kind == "text":
            cg.add(port.set_text(index, ent))
        else:
            cg.add(port.set_problem(ent))


async def light_telemetry_to_code(light_var, config, channels_mask):
    """A light on a receiver with several lights (its hub's telemetry, light_entities on): its estimated power /
    energy and a status, on the light's own device (or named after the light on the hub's). Created here, not in the
    light's schema: whether its board has several lights is known only from the whole configuration."""
    hub_conf = next(
        (h for h in CORE.config.get("dig2analog_ultra", []) or [] if h[CONF_ID] == config["dig2analog_ultra_id"]),
        None,
    )
    if hub_conf is None or CONF_TELEMETRY not in hub_conf:
        return
    tel = hub_conf[CONF_TELEMETRY]
    board = config["board"]
    if not tel[CONF_LIGHT_ENTITIES] or board >= len(tel[CONF_RECEIVERS]):
        return
    if lights_per_board(CORE.config, config["dig2analog_ultra_id"]).get(board, 0) < 2:
        return   # one light: the receiver's own measured power is the light's
    light_id = str(config[CONF_ID])
    device = config.get(CONF_DEVICE_ID)
    var = cg.new_Pvariable(ID(f"{light_id}_telemetry", is_declaration=True, type=UltraLightTelemetry))
    cg.add(var.set_hub(await cg.get_variable(config["dig2analog_ultra_id"])))
    cg.add(var.set_board(board))
    cg.add(var.set_channels(channels_mask))
    for key, (kind, name, schema) in LIGHT_ENTITIES.items():
        conf = {CONF_ID: f"{light_id}_{key}"}
        conf[CONF_NAME] = name if device is not None else f"{config.get(CONF_NAME, light_id)} {name[0].lower()}{name[1:]}"
        if device is not None:
            conf[CONF_DEVICE_ID] = device.id
        ent = await _new(kind, schema(conf))
        cg.add(getattr(var, f"set_{key}")(ent))
    cg.add(var.attach())
