"""The line's `telemetry:` (QuinLED Diff-Power Ultra receivers report back): its schema, the entities of each receiver
(a Home Assistant device of its own, `device_id`), the port's summary, and the lights' estimates. Used by __init__.py
and light.py; the logic is ultra_telemetry.h, the entities ultra_receiver.h."""

import logging

from esphome import pins
import esphome.codegen as cg
from esphome.components import binary_sensor, button, sensor, text_sensor, uart
from esphome.components import time as time_
import esphome.config_validation as cv
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_DEVICE_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_FILTERS,
    CONF_ID,
    CONF_NAME,
    CONF_RX_BUFFER_SIZE,
    CONF_RX_PIN,
    CONF_TIME_ID,
    CONF_UART_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DATA_RATE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_IDENTIFY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_CONFIG,
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
UltraTotals = ns.class_("UltraTotals", cg.Component)
UltraIdentifyButton = ns.class_("UltraIdentifyButton", button.Button)
UltraLineButton = ns.class_("UltraLineButton", button.Button)

CONF_TELEMETRY = "telemetry"
CONF_RECEIVERS = "receivers"
CONF_DIRECTION_PIN = "direction_pin"
CONF_RECEIVER_ENABLE_PIN = "receiver_enable_pin"
CONF_POLL_INTERVALS = "poll_intervals"
CONF_DIAGNOSTICS = "diagnostics"
CONF_LIGHT_ENTITIES = "light_entities"
CONF_ADAPTIVE_RATE = "adaptive_rate"
CONF_COMMISSIONING = "commissioning"
CONF_COMMISSION_MIN_VOLTAGE = "commission_min_voltage"
CONF_PORT_ID = "port_id"
CONF_PORT_LIMIT = "port_limit"
CONF_SUPPLY_VOLTAGE = "supply_voltage"
CONF_LOAD_WARNING = "load_warning"
CONF_LOAD_PROBLEM = "load_problem"
CONF_LOAD_PROBLEM_TIME = "load_problem_time"
CONF_UPDATE_RATE = "update_rate"
# the update rate: STATUS (voltage, current, power) and the live values (R13)
UPDATE_RATES = {"fast": 1000, "normal": 2000, "slow": 5000}
# receiver_enable_pin: direction_pin: DE and /RE tied to one pin (it is the receiver enable too)
RX_TIED = "direction_pin"
POLL_PAGES = ["status", "power", "energy", "supply", "health", "lifetime", "diag"]
# the UART's baud rate is the rate the receivers answer at (ULTRA_RW_RATE_*)
RW_RATES = {1000000: 0, 500000: 1, 250000: 2}
RW_RX_BUFFER_MIN = 1024
# none: Power, Energy, Status (+ its detail), Problem; basic: + the shown diagnostics; full (default): + the hidden ones
# (disabled in Home Assistant until enabled; ~0.8 KB of the ESP32's RAM per receiver)
DIAG_LEVELS = ["none", "basic", "full"]
# the receiver texts whose builders are compiled only when a receiver has one (USE_DIG2ANALOG_ULTRA_DIAG_TEXTS)
DIAG_TEXTS = ("last_reset", "last_trip", "last_dip", "channel_health")


def _value(unit, decimals, device_class=None, state_class=STATE_CLASS_MEASUREMENT, icon=None, diagnostic=True):
    kw = {"unit_of_measurement": unit, "accuracy_decimals": decimals, "state_class": state_class}
    if device_class:
        kw["device_class"] = device_class
    if icon:
        kw["icon"] = icon
    if diagnostic:
        kw["entity_category"] = ENTITY_CATEGORY_DIAGNOSTIC
    return sensor.sensor_schema(**kw)


def _live():
    """A live power twin (R25): W, no state class (no statistics), its name ends in "live power": one recorder glob
    (sensor.*_live_power) keeps every twin out of Home Assistant's database"""
    return sensor.sensor_schema(unit_of_measurement=UNIT_WATT, accuracy_decimals=1, device_class=DEVICE_CLASS_POWER)


def _count(icon):
    return sensor.sensor_schema(
        icon=icon, accuracy_decimals=0, state_class=STATE_CLASS_TOTAL_INCREASING, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    )


def _text(icon, diagnostic=True):
    if diagnostic:
        return text_sensor.text_sensor_schema(icon=icon, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
    return text_sensor.text_sensor_schema(icon=icon)


# A receiver's entities: key -> (kind, index in its C++ list, level, name, schema). Level 0: shown on its device (the
# few that matter every day); 1: diagnostics, shown; 2: diagnostics, disabled in Home Assistant until enabled. Every
# number is paced for Home Assistant's database (ultra_pacing.h); a user's own `filters:` replace that.
RECEIVER_ENTITIES = {
    # what it draws, measured (W): the mean of each 10 s (recorded), and live at the update rate (not recorded: R25);
    # its lifetime energy (Wh, counting up: the energy dashboard)
    "power": ("sensor", 0, 0, "Power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "live_power": ("sensor", 17, 0, "Live power", _live()),
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
    # (with a device_id) its port and position, "Port 4 · receiver 2": Home Assistant groups receivers by it
    "port": ("text", 8, 0, "Port", _text("mdi:ethernet")),
    # it blinks its identify pattern for 10 s: find it on the ceiling
    "identify": (
        "button", 0, 0, "Identify",
        button.button_schema(UltraIdentifyButton, device_class=DEVICE_CLASS_IDENTIFY, entity_category=ENTITY_CATEGORY_CONFIG),
    ),
}
# (only on a receiver's own device)
DEVICE_ONLY = {"port"}

# The port's summary (on the hub's device, or the telemetry's own device_id): key -> (kind, index, name, schema)
PORT_ENTITIES = {
    # at the sender: its voltage x the port current (without a supply_voltage: learned, else the receivers' power)
    "port_power": ("sensor", 0, "power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "port_live_power": ("sensor", 6, "live power", _live()),
    # its energy at the sender (Wh: kept across reboots, never down): the receivers' upstream in the energy dashboard
    "port_energy": (
        "sensor", 7, "energy",
        _value(UNIT_WATT_HOURS, 0, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING, diagnostic=False),
    ),
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
    # the line's buttons (disabled until enabled in Home Assistant): its current calibration (the whole line goes
    # dark ~1 s per board), its boards blink their firmware version
    "port_calibrate_current": (
        "button", 0, "calibrate current",
        button.button_schema(UltraLineButton, icon="mdi:tune-vertical", entity_category=ENTITY_CATEGORY_CONFIG),
    ),
    "port_show_version": (
        "button", 1, "show firmware version",
        button.button_schema(UltraLineButton, icon="mdi:numeric", entity_category=ENTITY_CATEGORY_CONFIG),
    ),
    # the line's telemetry health (diagnostics: full; disabled until enabled; once a minute): its poll windows, the
    # answers missing (% of those asked for in that minute), broken (bad CRC) or with contention, its receivers'
    # restarts, the line let go of late (no answers that window), the rate the receivers answer at (adaptive_rate)
    "port_poll_windows": ("health", 0, "poll windows", _count("mdi:swap-horizontal")),
    "port_missing_answers": (
        "health", 1, "missing answers", _value(UNIT_PERCENT, 1, icon="mdi:access-point-network-off")
    ),
    "port_broken_answers": ("health", 2, "broken answers", _count("mdi:message-alert-outline")),
    "port_answer_contention": ("health", 3, "answer contention", _count("mdi:call-merge")),
    "port_receiver_restarts": ("health", 4, "receiver restarts", _count("mdi:restart")),
    "port_late_releases": ("health", 5, "late releases", _count("mdi:timer-alert-outline")),
    "port_reply_rate": (
        "health", 6, "reply rate", _value("kbit/s", 0, DEVICE_CLASS_DATA_RATE, icon="mdi:speedometer")
    ),
    # the commissioning (commissioning: true; wiki: Commissioning): its buttons (config), where it is, the last run's
    # report, and whether the line is commissioned (the stored Finalize passed, the same receivers since)
    "port_commission": (
        "button", 2, "commission",
        button.button_schema(
            UltraLineButton, icon="mdi:clipboard-check-outline", entity_category=ENTITY_CATEGORY_CONFIG
        ),
    ),
    "port_finalize": (
        "button", 3, "finalize",
        button.button_schema(UltraLineButton, icon="mdi:check-decagram", entity_category=ENTITY_CATEGORY_CONFIG),
    ),
    "port_commissioning": ("comm_status", 0, "commissioning", _text("mdi:clipboard-text-outline")),
    "port_commissioning_report": ("comm_report", 0, "commissioning report", _text("mdi:text-box-search-outline")),
    "port_commissioned": (
        "comm_binary", 0, "commissioned",
        binary_sensor.binary_sensor_schema(
            icon="mdi:check-decagram-outline", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    ),
}
# (the commissioning's entities: only with commissioning: true, the default)
PORT_COMMISSION = {"port_commission", "port_finalize", "port_commissioning", "port_commissioning_report",
                   "port_commissioned"}
# (the health entities: with diagnostics full, or when the YAML lists one)
PORT_HEALTH = {key for key, spec in PORT_ENTITIES.items() if spec[0] == "health"}
PORT_DISABLED = {"port_calibrate_current", "port_show_version"} | PORT_HEALTH

# The controller's totals (its own device: every port's power at its sender, their energy), made once
TOTAL_ENTITIES = {
    "total_power": (0, "Total power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "total_live_power": (1, "Total live power", _live()),
    "total_energy": (
        2, "Total energy", _value(UNIT_WATT_HOURS, 0, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING, diagnostic=False)
    ),
}

# The estimates of a light on a receiver with several lights (on the light's own device): key -> (kind, name, schema)
LIGHT_ENTITIES = {
    "estimated_power": ("sensor", "Estimated power", _value(UNIT_WATT, 1, DEVICE_CLASS_POWER, diagnostic=False)),
    "estimated_energy": (
        "sensor", "Estimated energy",
        _value(UNIT_WATT_HOURS, 1, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING, diagnostic=False),
    ),
    "status": ("text", "Status", _text("mdi:list-status", diagnostic=False)),
    # (on the light's own device) its receiver: "Port 4 · receiver 2"
    "port": ("text", "Port", _text("mdi:ethernet")),
}


def ledger_key(s):
    """The lifetime ledger's key of a line or a light (ultra_ledger.h ledger_key: FNV-1a, 32 bit)"""
    h = 2166136261
    for c in str(s).encode("utf-8"):
        h ^= c
        h = (h * 16777619) & 0xFFFFFFFF
    return h


def _receiver_enable(value):
    if isinstance(value, str) and value.strip().lower() == RX_TIED:
        return RX_TIED
    return pins.gpio_output_pin_schema(value)


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
        # the sender port's current limit (3 x 750 mA PTCs), the load's warning and problem (early: PTCs trip easily)
        cv.Optional(CONF_PORT_LIMIT, default="2.25A"): cv.All(cv.current, cv.Range(min=0.1, max=20.0)),
        cv.Optional(CONF_LOAD_WARNING, default="80%"): cv.percentage,
        cv.Optional(CONF_LOAD_PROBLEM, default="90%"): cv.percentage,
        cv.Optional(CONF_LOAD_PROBLEM_TIME, default="30s"): cv.positive_time_period_milliseconds,
        # the sender's voltage (power at the sender, the first cable's loss); without it: receiver 1's highest
        cv.Optional(CONF_SUPPLY_VOLTAGE): cv.All(cv.voltage, cv.Range(min=5.0, max=60.0)),
        # estimated power / energy and a status per light, for the lights of a receiver with several lights
        cv.Optional(CONF_LIGHT_ENTITIES, default=True): cv.boolean,
        # the hub's RS-485 driver enable (high = the hub drives the line): let go of right after each poll frame, so
        # the receivers can answer. Leave it out when the hardware lets go of the line by itself.
        cv.Optional(CONF_DIRECTION_PIN): pins.internal_gpio_output_pin_schema,
        # the port's RS-485 receiver enable (high = its output drives the UART's RX; `inverted: true` for a /RE): on
        # from this line's poll frame until its window is read. With it on every line of a UART, several lines share
        # that UART (one window at a time); without, each line needs a UART of its own.
        # (receiver_enable_pin: direction_pin: DE and /RE tied to the direction pin; it drives the line again as
        # soon as its window is read, which turns its receiver off: such lines share a UART too)
        cv.Optional(CONF_RECEIVER_ENABLE_PIN): _receiver_enable,
        # how often the receivers' voltage, current and power are read, and the live values sent (Live power): fast
        # 1 s, normal 2 s, slow 5 s (the recorded values are 10 s means either way)
        cv.Optional(CONF_UPDATE_RATE, default="fast"): cv.one_of(*UPDATE_RATES, lower=True),
        # the receivers answer one rate slower (1 M -> 500 k -> 250 k) once broken answers keep coming (4 of the last
        # 32 windows), faster again after an hour without (never above the UART's baud_rate); false pins that rate
        cv.Optional(CONF_ADAPTIVE_RATE, default=True): cv.boolean,
        # the commissioning (wiki: Commissioning): Commission / Finalize buttons, its texts; its load test fails when a
        # receiver would get less than commission_min_voltage of the sender's voltage at the port's full load. The
        # stored date: time_id, else the only time component (none: "date unknown")
        cv.Optional(CONF_COMMISSIONING, default=True): cv.boolean,
        cv.Optional(CONF_COMMISSION_MIN_VOLTAGE, default="90%"): cv.All(cv.percentage, cv.Range(min=0.5, max=0.99)),
        cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
        cv.Optional(CONF_POLL_INTERVALS, default={}): cv.Schema(
            dict(
                [
                    # STATUS: voltage, current, temperature, the power state (default: the update rate)
                    (cv.Optional("status"), cv.All(cv.positive_time_period_milliseconds,
                                                   cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=60)))),
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
            if key in DEVICE_ONLY and device is None:
                item.pop(key, None)
                continue
            label = name if device is not None else f"{port} R{k + 1} {name[0].lower()}{name[1:]}"
            item[key] = _entity(item.get(key), label, device, ent_level == 2)
        out.append(item)
    tel[CONF_RECEIVERS] = out
    device = tel.get(CONF_DEVICE_ID)
    commissioning = tel.get(CONF_COMMISSIONING, True) is not False
    for key, (_kind, _index, name, _schema) in PORT_ENTITIES.items():
        if key in PORT_HEALTH and key not in tel and lvl < 2:
            continue
        if key in PORT_COMMISSION and not commissioning:
            tel.pop(key, None)
            continue
        label = f"{name[0].upper()}{name[1:]}" if device is not None else f"{port} {name}"
        tel[key] = _entity(tel.get(key), label, device, key in PORT_DISABLED)
    config[CONF_TELEMETRY] = tel
    return config


# The ESP32 RAM a configuration's lines take, roughly (measured on a classic ESP32, rounds 9-10): per receiver with
# its entities by diagnostics level, per light with its share of its line's state, per estimated light (a receiver
# with several lights), a line's own, the UART buffers, the lifetime ledger's reserve (its own sizing rule).
_LOGGER = logging.getLogger(__name__)
RAM_PER_RECEIVER = {"full": 1600, "basic": 900, "none": 600}
RAM_PER_LIGHT = 900
RAM_PER_ESTIMATE = 200
RAM_PER_LINE = 1000
RAM_PER_ENTITY = 60   # (a line's health sensors: round 12, from the full - basic difference per receiver entity)
RAM_PER_COMMISSION = 900   # a line's commissioning: 5 entities, its record (148 B), its texts, its state
RAM_LEDGER_ENTRY, RAM_LEDGER_POSITION = 80, 40
RAM_WARN_CLASSIC = 60 * 1024   # a classic ESP32: ~100-150 KB free with Wi-Fi and the API


def ram_estimate(full):
    """(bytes, parts) of every dig2analog_ultra line in the configuration."""
    hubs = full.get("dig2analog_ultra", []) or []
    parts = {"lines": RAM_PER_LINE * len(hubs), "receivers": 0, "lights": 0, "estimates": 0, "uarts": 0,
             "ledger": 0}
    counters = 0   # the ledger's positions: a receiver each, a port each, an estimated light each
    uarts = set()
    for conf in hubs:
        counts = lights_per_board(full, conf[CONF_ID])
        parts["lights"] += RAM_PER_LIGHT * sum(counts.values())
        tel = conf.get(CONF_TELEMETRY)
        if not tel:
            continue
        recs = tel.get(CONF_RECEIVERS) or []
        level = str(tel.get(CONF_DIAGNOSTICS, "full")).lower()
        parts["receivers"] += RAM_PER_RECEIVER.get(level, RAM_PER_RECEIVER["full"]) * len(recs)
        parts["lines"] += RAM_PER_ENTITY * len(PORT_HEALTH & set(tel))
        if tel.get(CONF_COMMISSIONING, True):
            parts["lines"] += RAM_PER_COMMISSION
        counters += len(recs) + 1
        if tel.get(CONF_LIGHT_ENTITIES, True):
            est = sum(n for b, n in counts.items() if b < len(recs) and n > 1)
            parts["estimates"] += RAM_PER_ESTIMATE * est
            counters += est
        uid = str(tel.get(CONF_UART_ID))
        if uid not in uarts:
            uarts.add(uid)
            u = uart_of(full, tel.get(CONF_UART_ID))
            parts["uarts"] += int(u.get(CONF_RX_BUFFER_SIZE, RW_RX_BUFFER_MIN)) if u else RW_RX_BUFFER_MIN
    if counters:
        pos = (counters + 31) // 32 * 32
        room = (counters + max(4, counters // 2) + 31) // 32 * 32
        parts["ledger"] = room * RAM_LEDGER_ENTRY + pos * RAM_LEDGER_POSITION
    return sum(parts.values()), parts


def warn_ram(full):
    """A warning when the lines would leave a classic ESP32 short of heap."""
    from esphome.components import esp32

    total, parts = ram_estimate(full)
    if esp32.get_esp32_variant() != esp32.VARIANT_ESP32 or total <= RAM_WARN_CLASSIC:
        return total
    detail = ", ".join(f"{k} {v / 1024:.1f}" for k, v in parts.items() if v)
    _LOGGER.warning(
        "dig2analog_ultra: these lines take about %.0f KB of the ESP32's RAM (KB: %s). A classic ESP32 has about "
        "100-150 KB free with Wi-Fi and the API, so above ~%d KB it may run short (a crash under load, a failed OTA). "
        "Use diagnostics: basic on the big lines (0.7 KB less per receiver), fewer lines per ESP32, or an ESP32-S3 / "
        "P4.",
        total / 1024,
        detail,
        RAM_WARN_CLASSIC // 1024,
    )
    return total


def uart_of(full, uart_id):
    for u in full.get("uart", []) or []:
        if u[CONF_ID] == uart_id:
            return u
    return None


def validate_telemetry_lines(full):
    """Across the lines: a UART carries one line's telemetry, or several lines' when each of them has a
    receiver_enable_pin (only the port whose window is open drives the UART's RX; the hubs take turns: one window
    per UART at a time). Two ports with one name or one device: ESPHome's own check refuses their entities' names
    (unique per device)."""
    hubs = full.get("dig2analog_ultra", []) or []
    uarts = {}
    for conf in hubs if isinstance(hubs, list) else [hubs]:
        tel = conf.get(CONF_TELEMETRY)
        if tel is None:
            continue
        uarts.setdefault(str(tel[CONF_UART_ID]), []).append(conf)
    for uart_id, confs in uarts.items():
        if len(confs) < 2:
            continue
        missing = [str(c[CONF_ID]) for c in confs if CONF_RECEIVER_ENABLE_PIN not in c[CONF_TELEMETRY]]
        if missing:
            names = " and ".join(f"'{c[CONF_ID]}'" for c in confs)
            raise cv.Invalid(
                f"the telemetry of {names} use the same uart '{uart_id}': lines share a UART only when each has a "
                f"receiver_enable_pin (its port's RS-485 receiver, on only during its own window); "
                f"{', '.join(repr(m) for m in missing)} {'has' if len(missing) == 1 else 'have'} none. Or give each "
                "line a UART of its own"
            )


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
    if tel.get(CONF_RECEIVER_ENABLE_PIN) == RX_TIED and CONF_DIRECTION_PIN not in tel:
        raise cv.Invalid(
            f"telemetry of '{conf[CONF_ID]}': receiver_enable_pin: direction_pin (DE and /RE tied) needs a direction_pin"
        )
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
    if kind in ("sensor", "health"):
        return await sensor.new_sensor(conf)
    if kind in ("text", "comm_status", "comm_report"):
        return await text_sensor.new_text_sensor(conf)
    if kind == "button":
        return await button.new_button(conf)
    return await binary_sensor.new_binary_sensor(conf)


async def _totals_to_code():
    """The controller's totals, once (the first line with telemetry)"""
    if CORE.data.get("dig2analog_ultra_totals"):
        return
    CORE.data["dig2analog_ultra_totals"] = True
    var = cg.new_Pvariable(ID("dig2analog_ultra_totals", is_declaration=True, type=UltraTotals))
    CORE.component_ids.add("dig2analog_ultra_totals")   # (made here, not in the configuration)
    await cg.register_component(var, {})
    for key, (index, name, schema) in TOTAL_ENTITIES.items():
        ent = await sensor.new_sensor(schema({CONF_ID: f"dig2analog_ultra_{key}", CONF_NAME: name}))
        cg.add(var.set_value(index, ent))


async def telemetry_to_code(hub, config):
    """The hub's telemetry: the poller, the receivers' entities, the port's summary."""
    tel = config[CONF_TELEMETRY]
    cg.add_define("USE_DIG2ANALOG_ULTRA_TELEMETRY")
    iv = dict(tel[CONF_POLL_INTERVALS])
    iv.setdefault("status", cv.TimePeriod(milliseconds=UPDATE_RATES[tel[CONF_UPDATE_RATE]]))
    receivers = tel[CONF_RECEIVERS]
    cg.add(
        hub.set_telemetry(
            await cg.get_variable(tel[CONF_UART_ID]),
            len(receivers),
            RW_RATES[uart_of(CORE.config, tel[CONF_UART_ID])[CONF_BAUD_RATE]],
            *[int(iv[k].total_milliseconds) for k in POLL_PAGES],
        )
    )
    cg.add(hub.set_telemetry_name(tel[CONF_NAME]))
    if not tel[CONF_ADAPTIVE_RATE]:
        cg.add(hub.set_telemetry_adaptive_rate(False))
    if tel[CONF_COMMISSIONING]:
        cg.add(hub.set_commissioning(tel[CONF_COMMISSION_MIN_VOLTAGE]))
        clock = tel.get(CONF_TIME_ID)
        times = CORE.config.get("time", []) or []
        if clock is None and len(times) == 1:
            clock = times[0][CONF_ID]
        if clock is not None:
            cg.add(hub.set_commission_clock(await cg.get_variable(clock)))
    cg.add(hub.set_telemetry_key(ledger_key(config[CONF_ID].id)))
    if CONF_DIRECTION_PIN in tel:
        cg.add(hub.set_telemetry_direction_pin(await cg.gpio_pin_expression(tel[CONF_DIRECTION_PIN])))
    if tel.get(CONF_RECEIVER_ENABLE_PIN) == RX_TIED:
        cg.add(hub.set_telemetry_receiver_tied(True))
    elif CONF_RECEIVER_ENABLE_PIN in tel:
        cg.add(hub.set_telemetry_receiver_enable_pin(await cg.gpio_pin_expression(tel[CONF_RECEIVER_ENABLE_PIN])))
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
                if CONF_FILTERS in rec[key]:
                    cg.add(var.set_unpaced(index))   # (the user's own filters replace the pacing)
            elif kind == "text":
                cg.add(var.set_text(index, ent))
                if key in DIAG_TEXTS:
                    cg.add_define("USE_DIG2ANALOG_ULTRA_DIAG_TEXTS")   # (their builders: only when one is there)
            elif kind == "button":
                cg.add(ent.set_hub(hub))
                cg.add(ent.set_board(k))
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
            if CONF_FILTERS in tel[key]:
                cg.add(port.set_unpaced(index))
        elif kind == "health":
            cg.add(port.set_health(index, ent))
        elif kind == "comm_status":
            cg.add(port.set_commission_status(ent))
        elif kind == "comm_report":
            cg.add(port.set_commission_report(ent))
        elif kind == "comm_binary":
            cg.add(port.set_commissioned(ent))
        elif kind == "text":
            cg.add(port.set_text(index, ent))
        elif kind == "button":
            cg.add(ent.set_hub(hub))
            cg.add(ent.set_kind(index))
        else:
            cg.add(port.set_problem(ent))
    await _totals_to_code()


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
    cg.add(var.set_key(ledger_key(light_id)))   # (its estimated energy kept in the ledger)
    for key, (kind, name, schema) in LIGHT_ENTITIES.items():
        if key == "port" and device is None:
            continue   # (on the light's own device only)
        conf = {CONF_ID: f"{light_id}_{key}"}
        conf[CONF_NAME] = name if device is not None else f"{config.get(CONF_NAME, light_id)} {name[0].lower()}{name[1:]}"
        if device is not None:
            conf[CONF_DEVICE_ID] = device.id
        ent = await _new(kind, schema(conf))
        cg.add(getattr(var, f"set_{key}")(ent))
    cg.add(var.attach())
