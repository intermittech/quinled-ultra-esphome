"""QuinLED Diff-Power sender ports: each port's 55 V switched by an enable output (and the sender's board detection),
in standby while nothing on it is lit (wiki: Diff-Power ports). The logic is diffpower_core.h; any ESPHome light can
be on a port, and a dig2analog_ultra hub on it (its `port:`) adds its lights itself.

(ESPHome's power_supply can't be the base: its PowerSupply is final, its requests aren't virtual, and its enable_time
blocks the loop.)"""

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import binary_sensor, button, light, select, sensor, sun, text_sensor
from esphome.components import time as time_
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEVICE_ID,
    CONF_ID,
    CONF_LATITUDE,
    CONF_LONGITUDE,
    CONF_NAME,
    CONF_TIME_ID,
    CONF_TIMEOUT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_HOUR,
)
from esphome.core import CORE
import esphome.final_validate as fv

CODEOWNERS = ["@intermittech"]
MULTI_CONF = True


AUTO_LOAD = ["binary_sensor", "button", "light", "select", "sensor", "text_sensor"]


diffpower_ns = cg.esphome_ns.namespace("diffpower")
DiffPowerPort = diffpower_ns.class_("DiffPowerPort", cg.Component)
PowerModeSelect = diffpower_ns.class_("PowerModeSelect", select.Select, cg.Component)
WakeAction = diffpower_ns.class_("WakeAction", automation.Action)
WakeButton = diffpower_ns.class_("WakeButton", button.Button)

CONF_ENABLE_PIN = "enable_pin"
CONF_DETECT_PIN = "detect_pin"
CONF_POWER_MODE = "power_mode"
CONF_STANDBY_AFTER = "standby_after"
CONF_HOTPLUG_DELAY = "hotplug_delay"
CONF_BOOT_MARGIN = "boot_margin"
CONF_NOTICE_TIME = "notice_time"
CONF_MIN_OFF_TIME = "min_off_time"
CONF_MIN_ON_TIME = "min_on_time"
CONF_SUN_ID = "sun_id"
CONF_SUN_ENTITY = "sun_entity"
CONF_DARK_BEFORE = "dark_before"
CONF_DARK_AFTER = "dark_after"
CONF_TIME_UNKNOWN = "time_unknown"
CONF_LIGHTS = "lights"
CONF_PORTS = "ports"
CONF_STATUS = "status"
CONF_PROBLEM = "problem"
CONF_STANDBY_CYCLES = "standby_cycles"
CONF_STANDBY_HOURS = "standby_hours"
CONF_DETECT_DROPS = "detect_drops"
CONF_POWER_MODE_SELECT = "power_mode_select"
CONF_WAKE = "wake"
CONF_WAKE_TIMEOUT = "wake_timeout"

# PowerMode (diffpower_core.h), and the select's options in that order
POWER_MODES = {"standby": 0, "during_dark": 1, "always_on": 2}
MODE_OPTIONS = ["Standby", "During dark", "Always on"]
# During dark without sun times (no valid time yet; Home Assistant's missing or stale and no sun_id)
TIME_UNKNOWN = {"always_on": True, "standby": False}


# a turn-off period (the port's, a light's): 24 h at most (it counts in ms, in 32 bits)
HOLD = cv.All(cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(hours=24)))


def _light_entry(value):
    """A light on the port: its id, or {id, standby_after} (its own turn-off period)."""
    if isinstance(value, dict):
        return cv.Schema(
            {
                cv.Required(CONF_ID): cv.use_id(light.LightState),
                cv.Optional(CONF_STANDBY_AFTER): HOLD,
            }
        )(value)
    return {CONF_ID: cv.use_id(light.LightState)(value)}


# the port's entities: key -> (kind, its name after the port's, schema)
PORT_ENTITIES = {
    # "No board", "Standby", "Waking", "On", "Powering off", "Connection lost (N drops)"
    CONF_STATUS: ("text", "power state", text_sensor.text_sensor_schema(icon="mdi:power-plug-outline")),
    # on while the board's detect is gone after it went away powered (unplugged, a cable fault, the supply gone), or
    # while a light wants a port without a board
    CONF_PROBLEM: (
        "binary",
        "power problem",
        binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_PROBLEM, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    ),
    # the port's planned power-offs (kept on the ESP32 across reboots, as the next two)
    CONF_STANDBY_CYCLES: (
        "sensor",
        "standby cycles",
        sensor.sensor_schema(
            icon="mdi:power-sleep",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    # its time in standby
    CONF_STANDBY_HOURS: (
        "sensor",
        "hours in standby",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            icon="mdi:sleep",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    # its board's detect lost while powered
    CONF_DETECT_DROPS: (
        "sensor",
        "detect drops",
        sensor.sensor_schema(
            icon="mdi:connection",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    # Standby / During dark / Always on (restored on the ESP32; the YAML's power_mode first)
    CONF_POWER_MODE_SELECT: (
        "select",
        "power mode",
        select.select_schema(PowerModeSelect, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:power-settings").extend(
            cv.COMPONENT_SCHEMA
        ),
    ),
    # a press wakes the port for wake_timeout (each press again from then on): motion automations press it
    CONF_WAKE: ("button", "wake", button.button_schema(WakeButton, icon="mdi:power-plug")),
}


def _defaults(config):
    """Every entity of the port: named after it ("Port 1 power state", ...), or on its own device_id without the
    prefix ("Power state", "Wake", ...: the device's name is in front); an entity set to false is left out."""
    if not isinstance(config, dict):
        return config
    config = dict(config)
    port = str(config.get(CONF_NAME, "Port"))
    for key, (_kind, name, _schema) in PORT_ENTITIES.items():
        value = config.get(key, {})
        if value is False:
            config.pop(key, None)
            continue
        ent = dict(value or {})
        if CONF_DEVICE_ID in config:
            ent.setdefault(CONF_NAME, f"{name[0].upper()}{name[1:]}")
            ent.setdefault(CONF_DEVICE_ID, config[CONF_DEVICE_ID])
        else:
            ent.setdefault(CONF_NAME, f"{port} {name}")
        config[key] = ent
    return config


def _sun_conf(full, sun_id):
    """The sun component sun_id's configuration (its latitude / longitude: sun::Sun keeps them to itself)."""
    confs = full.get("sun", [])
    for conf in confs if isinstance(confs, list) else [confs]:
        if conf.get(CONF_ID) == sun_id:
            return conf
    raise cv.Invalid(f"diffpower: sun '{sun_id}' not found")


def _dark_plan(config, full):
    """Where a port's "During dark" comes from: (its clock's id or None, Home Assistant's sun times, the sun's
    configuration or None). The clock: time_id, else the sun's, else the only time component. Home Assistant's sun
    times: with api: and a clock (ESPHome's sun the backup)."""
    sun_conf = _sun_conf(full, config[CONF_SUN_ID]) if CONF_SUN_ID in config else None
    clock = config.get(CONF_TIME_ID)
    if clock is None and sun_conf is not None:
        clock = sun_conf[CONF_TIME_ID]
    times = full.get("time", [])
    if clock is None and len(times) == 1:
        clock = times[0][CONF_ID]
    return clock, "api" in full and clock is not None, sun_conf


def _mode_options(config, full):
    """The power mode select's options: "During dark" with a dark source."""
    _clock, ha, sun_conf = _dark_plan(config, full)
    return MODE_OPTIONS if ha or sun_conf is not None else [MODE_OPTIONS[0], MODE_OPTIONS[2]]


def _validate_dark(config, full):
    """power_mode: during_dark needs a dark source; sun_entity: one for all ports."""
    _clock, ha, sun_conf = _dark_plan(config, full)
    if config[CONF_POWER_MODE] == "during_dark" and not ha and sun_conf is None:
        if "api" not in full:
            msg = ("power_mode: during_dark needs the sunrise / sunset: from Home Assistant (api:, and a time: "
                   "component), or from ESPHome's sun component (sun_id: latitude, longitude)")
        elif not full.get("time"):
            msg = ("power_mode: during_dark needs the time now: add a time: component (e.g. platform: homeassistant), "
                   "or a sun_id")
        else:
            msg = "power_mode: during_dark: there are several time components, say which one with time_id"
        raise cv.Invalid(msg, path=[CONF_POWER_MODE])
    entities = {p.get(CONF_SUN_ENTITY) for p in full.get("diffpower", [])} - {None}
    if len(entities) > 1:
        raise cv.Invalid(f"sun_entity: one for all ports ({', '.join(sorted(entities))})", path=[CONF_SUN_ENTITY])


PORT_SCHEMA = cv.All(
    _defaults,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DiffPowerPort),
            # the port's name: its entities are "<name> power state", "<name> power mode", ...
            cv.Required(CONF_NAME): cv.string_strict,
            cv.Optional(CONF_DEVICE_ID): cv.sub_device_id,
            # the sender's ChannelEnable_n (high: the port may have 55 V) and RemoteDetected_n (high: a board on it)
            cv.Required(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DETECT_PIN): pins.gpio_input_pin_schema,
            # standby (sleep after the turn-off period, wake on a command or a pre-wake), during_dark (also powered
            # from dark_before before sunset to dark_after after sunrise: Home Assistant's sun times, or the sun_id's),
            # always_on; the select changes it
            cv.Optional(CONF_POWER_MODE, default="standby"): cv.one_of(*POWER_MODES, lower=True),
            # the turn-off period: powered this long after its last light went off (a light may set its own; at least
            # min_on_time)
            cv.Optional(CONF_STANDBY_AFTER, default="10min"): HOLD,
            # the sender's hot-plug delay (enable AND detect -> 55 V), then the boards' boot: the hub's frames start
            # after both (a Diff-Power receiver waits 1.5 s for them before it shows its power-on scene)
            cv.Optional(CONF_HOTPLUG_DELAY, default="1500ms"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(seconds=10))
            ),
            cv.Optional(CONF_BOOT_MARGIN, default="200ms"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(seconds=10))
            ),
            # the planned power-off: the notice (3 copies at 0 / 50 / 100 ms), the enable low after notice_time, then
            # low for at least min_off_time (the bus falls below the receivers' UVLO) before any wake. 1 s at most: a
            # receiver forgets the notice 2 s after it, and the port gives up on copies it can't send after 1 s
            cv.Optional(CONF_NOTICE_TIME, default="200ms"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(min=cv.TimePeriod(milliseconds=150),
                                                               max=cv.TimePeriod(seconds=1))
            ),
            cv.Optional(CONF_MIN_OFF_TIME, default="1s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(seconds=30)),
            ),
            # powered at least this long once on: the floor of turn-off periods and pre-wakes (no wake storms)
            cv.Optional(CONF_MIN_ON_TIME, default="2min"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(hours=1))
            ),
            # the Wake button's: powered this long after a press if no light turns on (at least min_on_time)
            cv.Optional(CONF_WAKE_TIMEOUT, default="2min"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(hours=24))
            ),
            # "During dark": the sunrise / sunset from Home Assistant's sun entity (with api:), ESPHome's sun (sun_id:
            # latitude, longitude) as the backup, or alone without Home Assistant. The time now: time_id, else the
            # sun's, else the only time component.
            cv.Optional(CONF_SUN_ENTITY, default="sun.sun"): cv.entity_id,
            cv.Optional(CONF_SUN_ID): cv.use_id(sun.Sun),
            cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
            cv.Optional(CONF_DARK_BEFORE, default="60min"): cv.All(
                cv.positive_time_period_seconds, cv.Range(max=cv.TimePeriod(hours=12))
            ),
            cv.Optional(CONF_DARK_AFTER, default="60min"): cv.All(
                cv.positive_time_period_seconds, cv.Range(max=cv.TimePeriod(hours=12))
            ),
            # During dark without sun times: until the ESP32 has a valid time, or with Home Assistant's missing or
            # stale (more than 10 min past) and no sun_id
            cv.Optional(CONF_TIME_UNKNOWN, default="always_on"): cv.one_of(*TIME_UNKNOWN, lower=True),
            # the lights on this port (any ESPHome light; a dig2analog_ultra hub with `port:` adds its own)
            cv.Optional(CONF_LIGHTS, default=[]): cv.ensure_list(_light_entry),
            **{cv.Optional(key): spec[2] for key, spec in PORT_ENTITIES.items()},
        }
    ).extend(cv.COMPONENT_SCHEMA),
)

CONFIG_SCHEMA = PORT_SCHEMA


def _light_confs(full):
    return {str(c[CONF_ID].id): c for c in full.get("light", []) if CONF_ID in c}


def _partition_sources(conf):
    """The lights under a partition light's segments (addressable strips, or single lights)."""
    out = []
    for seg in conf.get("segments", []):
        src = seg.get(CONF_ID, seg.get("single_light_id"))
        if src is not None:
            out.append(str(src.id))
    return out


def _related_lights(full, entries, default_ms=0):
    """Lights that light the same pixels as the port's (a partition writes its strip's buffer without turning the
    strip's light on), at any depth: from each listed light down through partitions to the physical strips, then up
    through every partition over any of those (nested ones too; a cycle stops). id -> turn-off period in ms: the
    longest of the listed lights it shares pixels with (default_ms: the port's, for a light without its own)."""
    under = {pid: _partition_sources(conf) for pid, conf in _light_confs(full).items()
             if conf.get("platform") == "partition"}
    over = {}
    for pid, sources in under.items():
        for src in sources:
            over.setdefault(src, set()).add(pid)
    listed = {}
    for e in entries:
        hold = e.get(CONF_STANDBY_AFTER)
        listed[str(e[CONF_ID].id)] = int(hold.total_milliseconds) if hold else default_ms
    extra = {}
    for lid, hold in listed.items():
        down, stack = set(), [lid]
        while stack:
            x = stack.pop()
            if x not in down:
                down.add(x)
                stack.extend(under.get(x, ()))
        group, stack = set(down), list(down)
        while stack:
            for p in over.get(stack.pop(), ()):
                if p not in group:
                    group.add(p)
                    stack.append(p)
        for other in group - set(listed):
            extra[other] = max(extra.get(other, 0), hold)
    return extra


def _port_lights(full):
    """Every light some port powers: its `lights`, their partitions / strips, the lights of Ultra lines with `port:`."""
    lights = set()
    for port in full.get("diffpower", []):
        lights |= {str(e[CONF_ID].id) for e in port.get(CONF_LIGHTS, [])}
        lights |= set(_related_lights(full, port.get(CONF_LIGHTS, [])))   # (any depth)
    hubs = {str(h[CONF_ID].id) for h in full.get("dig2analog_ultra", []) if h.get("port") is not None}
    for lid, conf in _light_confs(full).items():
        hub = conf.get("dig2analog_ultra_id")
        if conf.get("platform") == "dig2analog_ultra" and hub is not None and str(hub.id) in hubs:
            lights.add(lid)
    return lights


def _final_validate(config):
    """During dark's source; diffpower.wake's lights: each one on a port (else the action does nothing for it)."""
    full = fv.full_config.get()
    _validate_dark(config, full)
    on_ports = _port_lights(full)
    for lid in sorted(CORE.data.get("diffpower_wake_lights", set())):
        if lid not in on_ports:
            raise cv.Invalid(
                f"diffpower.wake: light '{lid}' is on no Diff-Power port (list it in a port's lights:, or put its "
                "Ultra line on a port)"
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_DIFFPOWER")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_stats_key(str(config[CONF_ID].id)))
    cg.add(var.set_enable_pin(await cg.gpio_pin_expression(config[CONF_ENABLE_PIN])))
    if CONF_DETECT_PIN in config:
        cg.add(var.set_detect_pin(await cg.gpio_pin_expression(config[CONF_DETECT_PIN])))
    cg.add(
        var.set_timing(
            int(config[CONF_HOTPLUG_DELAY].total_milliseconds),
            int(config[CONF_BOOT_MARGIN].total_milliseconds),
            int(config[CONF_NOTICE_TIME].total_milliseconds),
            int(config[CONF_MIN_OFF_TIME].total_milliseconds),
            int(config[CONF_MIN_ON_TIME].total_milliseconds),
            int(config[CONF_STANDBY_AFTER].total_milliseconds),
        )
    )
    cg.add(var.set_unknown_on(TIME_UNKNOWN[config[CONF_TIME_UNKNOWN]]))
    cg.add(var.set_mode(POWER_MODES[config[CONF_POWER_MODE]]))
    clock, ha, sun_conf = _dark_plan(config, CORE.config)
    if ha or sun_conf is not None:
        cg.add_define("USE_DIFFPOWER_DARK")
        cg.add(
            var.set_dark(
                await cg.get_variable(clock),
                int(config[CONF_DARK_BEFORE].total_seconds),
                int(config[CONF_DARK_AFTER].total_seconds),
            )
        )
        if ha:   # (one subscription for all ports)
            cg.add_define("USE_DIFFPOWER_HA")
            cg.add_define("USE_API_HOMEASSISTANT_STATES")
            cg.add(var.set_sun_entity(config[CONF_SUN_ENTITY]))
        if sun_conf is not None:
            cg.add_define("USE_DIFFPOWER_SUN")
            cg.add(var.set_sun(float(sun_conf[CONF_LATITUDE]), float(sun_conf[CONF_LONGITUDE])))
    for entry in config[CONF_LIGHTS]:
        hold = int(entry[CONF_STANDBY_AFTER].total_milliseconds) if CONF_STANDBY_AFTER in entry else 0
        cg.add(var.add_light(await cg.get_variable(entry[CONF_ID]), hold))
    # the partitions of its strips and the strips under its partitions, at any depth: the same pixels
    default_ms = int(config[CONF_STANDBY_AFTER].total_milliseconds)
    confs = _light_confs(CORE.config)
    for lid, hold in sorted(_related_lights(CORE.config, config[CONF_LIGHTS], default_ms).items()):
        cg.add(var.add_light(await cg.get_variable(confs[lid][CONF_ID]), 0 if hold == default_ms else hold))
    if conf := config.get(CONF_STATUS):
        cg.add(var.set_status_sensor(await text_sensor.new_text_sensor(conf)))
    if conf := config.get(CONF_PROBLEM):
        cg.add(var.set_problem_sensor(await binary_sensor.new_binary_sensor(conf)))
    if conf := config.get(CONF_STANDBY_CYCLES):
        cg.add(var.set_cycles_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_STANDBY_HOURS):
        cg.add(var.set_hours_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_DETECT_DROPS):
        cg.add(var.set_drops_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_WAKE):
        btn = await button.new_button(conf)
        cg.add(btn.set_port(var))
        cg.add(btn.set_timeout(int(config[CONF_WAKE_TIMEOUT].total_milliseconds)))
    if conf := config.get(CONF_POWER_MODE_SELECT):
        sel = await select.new_select(conf, options=_mode_options(config, CORE.config))
        await cg.register_component(sel, conf)
        cg.add(sel.set_port(var))


# ------------------------------------------------------------------------------------------------ the wake action
def _validate_wake(config):
    if not config[CONF_PORTS] and not config[CONF_LIGHTS]:
        raise cv.Invalid("diffpower.wake: give ports, lights or both")
    # (its lights must be on a port: checked once the whole configuration is there)
    CORE.data.setdefault("diffpower_wake_lights", set()).update(str(l.id) for l in config[CONF_LIGHTS])
    return config


@automation.register_action(
    "diffpower.wake",
    WakeAction,
    cv.All(cv.Schema(
        {
            # the ports to wake, and / or lights (their ports)
            cv.Optional(CONF_PORTS, default=[]): cv.ensure_list(cv.use_id(DiffPowerPort)),
            cv.Optional(CONF_LIGHTS, default=[]): cv.ensure_list(cv.use_id(light.LightState)),
            # back to standby after this if no light turns on (at least the port's min_on_time)
            cv.Optional(CONF_TIMEOUT, default="2min"): cv.templatable(
                cv.All(cv.positive_time_period_milliseconds, cv.Range(max=cv.TimePeriod(hours=24)))
            ),
        }
    ), _validate_wake),
    synchronous=True,
)
async def wake_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    for p in config[CONF_PORTS]:
        cg.add(var.add_port(await cg.get_variable(p)))
    for l in config[CONF_LIGHTS]:
        cg.add(var.add_light(await cg.get_variable(l)))
    cg.add(var.set_timeout(await cg.templatable(config[CONF_TIMEOUT], args, cg.uint32)))
    return var
