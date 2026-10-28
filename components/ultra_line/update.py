"""Update all receivers on this port: one update entity per line (ultra_line) (fw_update.h).

The entity reads the firmware repo's signed channel manifest from GitHub and keeps the port's plan: every position's
immutable target from the receivers' own identity or from the inventory below, the release's image for each node,
family by family. Install is one port job; its per-node results stay in the report. Design: the protocol's
docs/update.md (Port job, Job results).
"""

import logging
from pathlib import Path
import re

from esphome import automation
import esphome.codegen as cg
from esphome.components import http_request, select, text_sensor, update
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import CONF_ID, CONF_SOURCE, DEVICE_CLASS_FIRMWARE, ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.core import CORE

from . import CONF_ULTRA_LINE_ID, PORT_MAX_BOARDS, UltraHub, ultra_line_ns
from .board_position import board_position, board_slot, out_of_range, positions_text

_LOGGER = logging.getLogger(__name__)

DEPENDENCIES = ["ultra_line", "http_request"]

CONF_CHANNEL = "channel"
CONF_CHANNEL_SELECT = "channel_select"
CONF_STATUS = "status"
CONF_REPORT = "report"
CONF_BLACKOUT = "blackout"
CONF_PUBLIC_KEYS = "public_keys"
CONF_VERIFY_SIGNATURE = "verify_signature"

# The QuinLED firmware repo on GitHub (its `channels` branch holds the signed channel manifests).
DEFAULT_REPO = "intermittech/quinled-ultra-firmware"

# FW_CH_* (fw_signature.h); the select's options are in this order
CHANNELS = {"stable": 1, "beta": 2, "alpha": 4}
# fw_keys.h: production (none until the firmware repo is set up) or the TEST keys (development, bench)
PUBLIC_KEYS = {
    "production": ("esphome::ultra_line::FW_PRODUCTION_KEYS", "esphome::ultra_line::FW_PRODUCTION_KEY_COUNT"),
    "test": (
        "esphome::ultra_line::FW_TEST_KEYS",
        "(sizeof(esphome::ultra_line::FW_TEST_KEYS) / sizeof(esphome::ultra_line::FW_TEST_KEYS[0]))",
    ),
}
def _hex_bytes(value, count):
    value = cv.string_strict(value)
    if not re.fullmatch(r"[0-9a-fA-F]{%d}" % (count * 2), value):
        raise cv.Invalid(f"expected {count * 2} hexadecimal digits")
    return value.lower()


def _check_boot_info(blob):
    """The exact 32-byte ABI-3 protected bootloader export: QDB2, a bootloader version, ABI byte 3, the entry bound."""
    if len(blob) != 32 or blob[:4] != b"QDB2" or blob[9] != 3 or not blob[4] or blob[24] < 5:
        raise cv.Invalid("boot_info must be the exact 32-byte ABI-3 protected bootloader export")
    return blob


def _boot_identity(key, export_hex):
    """One table entry. The key names one exact bootloader: <board>_rev<RR>_bl<N>, the board and revision it is built
    for and the bootloader version its export carries (an entry whose key says otherwise is refused)."""
    blob = _check_boot_info(bytes.fromhex(export_hex))
    if not key.endswith(f"_rev{blob[12]:02d}_bl{blob[4]}"):
        raise ValueError(f"boot identity '{key}': its export is revision {blob[12]}, bootloader {blob[4]}")
    return key, blob


# The protected boot identity (the 32-byte export of the firmware's tools/export_boot_info.py) of every bootloader that
# has shipped, so a board without a return channel needs `boot_identity: <key>` in the YAML instead of a `boot_info:`
# file. A new bootloader is one more entry, never a changed one (boards in the field still run the old one); the key is
# never guessed from a product name. A board with any other bootloader takes its own export through `boot_info:`.
BOOT_IDENTITIES = dict([
    # Dig2Analog-Plus-5ch rev 0.4, assembly variant 2, bootloader 4 (build 261030b): QDB2, bootloader 4, target (board 1,
    # V203, F8U6, layout 1, ABI 3, features 2, variant 2, revision 4, slot 0x08002100 + 57,088 B), geometry 88 0a 14,
    # entry 5 (x 10 ms), build id
    _boot_identity("dig2analog_plus_5ch_rev04_bl4",
                   "51444232040101010103020204080021000000df00880a140532363130333062"),
])


def _validate_inventory_item(item):
    if ("boot_info" in item) == ("boot_identity" in item):
        raise cv.Invalid("a board needs either its boot_identity (a shipped bootloader) or its boot_info file (the "
                         "jig's export): exactly one")
    layout = [k for k in ("metadata_base", "metadata_bytes", "bootloader_bytes") if k in item]
    if layout and len(layout) != 3:
        raise cv.Invalid("metadata_base, metadata_bytes and bootloader_bytes: all three or none (none: the signed "
                         "manifest's layout for the board's target)")
    if item["return_channel"] and "uid" not in item:
        raise cv.Invalid("return_channel: true needs the board's uid")
    return item


# A board the return channel can't read (no telemetry, behind the line's receivers, a board without a return path):
# its protected boot identity for its position (a shipped bootloader's by name, or the jig's boot-info export), and
# optionally its layout and entry blackout (otherwise the signed manifest's, for its exact target).
INVENTORY_SCHEMA = cv.All(
    cv.Schema({
        # its position (1 = the first board after the controller)
        cv.Required("board"): cv.All(board_position, cv.int_range(max=max(PORT_MAX_BOARDS.values()))),
        cv.Optional("boot_info"): cv.file_,
        cv.Optional("boot_identity"): cv.one_of(*BOOT_IDENTITIES, lower=True),
        cv.Optional("metadata_base"): cv.hex_uint32_t,
        cv.Optional("metadata_bytes"): cv.int_range(min=256, max=65536),
        cv.Optional("bootloader_bytes"): cv.int_range(min=1, max=1048576),
        cv.Optional("uid"): lambda value: _hex_bytes(value, 12),
        cv.Optional("return_channel", default=False): cv.boolean,
        cv.Optional(CONF_BLACKOUT): cv.All(cv.positive_time_period_milliseconds,
                                           cv.Range(max=cv.TimePeriod(seconds=60))),
    }),
    _validate_inventory_item,
)


def production_keys_provisioned():
    """fw_keys.h holds production keys (FW_PRODUCTION_KEY_COUNT > 0): QuinLED provisions them with the first release."""
    text = (Path(__file__).parent / "fw_keys.h").read_text(encoding="utf-8")
    m = re.search(r"FW_PRODUCTION_KEY_COUNT\s*=\s*(\d+)", text)
    return m is not None and int(m.group(1)) > 0

UltraFirmwareUpdate = ultra_line_ns.class_("UltraFirmwareUpdate", update.UpdateEntity, cg.PollingComponent)
FwChannelSelect = ultra_line_ns.class_("FwChannelSelect", select.Select, cg.Component)
AbortUpdateAction = ultra_line_ns.class_("AbortUpdateAction", automation.Action)


def AUTO_LOAD(config=None):
    """json for the manifest; select / text_sensor only for the entities that use them (no config: all)."""
    loads = ["json"]
    if not config or CONF_CHANNEL_SELECT in config:
        loads.append("select")
    if not config or CONF_STATUS in config or CONF_REPORT in config:
        loads.append("text_sensor")
    return loads


def _validate_source(value):
    """OWNER/REPO on GitHub (its `channels` branch), or an http(s) URL of a manifest ({channel} is replaced)."""
    value = cv.string_strict(value)
    if "://" in value:
        return cv.url(value)
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", value):
        raise cv.Invalid("source: OWNER/REPO on GitHub, or the http(s) URL of a manifest ({channel} is replaced)")
    return value


def _validate(config):
    if not config[CONF_VERIFY_SIGNATURE] and CONF_SOURCE not in config:
        raise cv.Invalid(
            f"{CONF_VERIFY_SIGNATURE}: false only with your own {CONF_SOURCE} (a LAN mirror or your own builds)"
        )
    return config


CONFIG_SCHEMA = cv.All(
    update.update_schema(UltraFirmwareUpdate, device_class=DEVICE_CLASS_FIRMWARE, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(
        {
            cv.GenerateID(CONF_ULTRA_LINE_ID): cv.use_id(UltraHub),
            cv.GenerateID(http_request.CONF_HTTP_REQUEST_ID): cv.use_id(http_request.HttpRequestComponent),
            cv.Optional(CONF_SOURCE): _validate_source,
            cv.Optional(CONF_CHANNEL, default="stable"): cv.enum(CHANNELS, lower=True),
            cv.Optional(CONF_PUBLIC_KEYS, default="production"): cv.one_of(*PUBLIC_KEYS, lower=True),
            cv.Optional(CONF_VERIFY_SIGNATURE, default=True): cv.boolean,
            # the boards the receivers' return channel can't read (see INVENTORY_SCHEMA)
            cv.Optional("inventory", default=[]): cv.ensure_list(INVENTORY_SCHEMA),
            # optional: switch the channel from Home Assistant (stored on the ESP32)
            cv.Optional(CONF_CHANNEL_SELECT): select.select_schema(
                FwChannelSelect, entity_category=ENTITY_CATEGORY_CONFIG, icon="mdi:source-branch"
            ).extend(cv.COMPONENT_SCHEMA),
            # optional: what the port's update does now
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:information-outline"
            ),
            # optional: the last job's per-node results (before the first one: the plan), kept on the ESP32
            cv.Optional(CONF_REPORT): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:clipboard-list-outline"
            ),
        }
    )
    .extend(cv.polling_component_schema("6h")),
    _validate,
)


def _final_validate(config):
    """One update entity per line (a port job owns it), the inventory's positions (the hub owns the port type and
    checks its board limit on every line)."""
    full = fv.full_config.get()
    hub = config[CONF_ULTRA_LINE_ID]
    if config[CONF_PUBLIC_KEYS] == "production" and config[CONF_VERIFY_SIGNATURE] and not production_keys_provisioned():
        _LOGGER.warning(
            "update '%s': this component has no production signing keys yet (QuinLED provisions them with the "
            "firmware's first release): until then no release installs. For bench builds: public_keys: test with "
            "your own source.",
            config.get("name", hub),
        )
    line = next((h for h in full.get("ultra_line", []) or [] if h[CONF_ID] == hub), {})
    boards = line.get("num_boards", 0)
    positions = [item["board"] for item in config["inventory"]]
    if len(set(positions)) != len(positions):
        raise cv.Invalid("update inventory: each board once")
    for b in positions:
        if b > boards:
            raise cv.Invalid("update inventory: " + out_of_range(b, hub, boards))
    # every board the return channel can't read must be provisioned: the receivers come first on the line
    telemetry = line.get("telemetry")
    heard = len(telemetry.get("receivers", [])) if telemetry else 0
    given = {board_slot(b) for b in positions}
    missing = [b for b in range(heard, boards) if b not in given]
    if missing:   # (the plan stays blocked with that reason in Home Assistant)
        plural = "s" if len(missing) > 1 else ""
        _LOGGER.warning(
            "update '%s': board%s %s of '%s' can't report its identity (no return channel to it): until the inventory "
            "gives its boot_identity (or the jig's boot_info export), the port's update stays blocked",
            config.get("name", hub), plural, positions_text(missing), hub,
        )
    same = [
        u
        for u in full.get("update", []) or []
        if u.get("platform") == "ultra_line" and u.get(CONF_ULTRA_LINE_ID) == hub
    ]
    if len(same) > 1:
        raise cv.Invalid(f"ultra_line '{hub}' has {len(same)} firmware update entities; one per line")
    # After a port update the line stays quiet (host quiet >= 2,100 ms), then the hub sends everything again. Boards
    # switch off without frames for signal_loss_timeout (sent in 100 ms): with off / fade_off it must outlast that
    # quiet, or every light of the line goes off after an update.
    for h in full.get("ultra_line", []) or []:
        if h[CONF_ID] != hub or h.get("signal_loss", "hold") == "hold":
            continue
        timeout = round(h["signal_loss_timeout"].total_milliseconds / 100) * 100
        if timeout < 2100:
            raise cv.Invalid(
                f"ultra_line '{hub}': with a firmware update entity and signal_loss: {h['signal_loss']}, "
                f"signal_loss_timeout ({timeout} ms) must be at least 2.1s: the line is quiet for 2.1 s after a port "
                f"update, and the boards would switch their lights off"
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_ULTRA_LINE_UPDATE")
    # the vendored sender library's port job takes up to a Dig2Analog-Plus-5ch pin's boards
    cg.add_build_flag(f"-DULTRA_SENDER_MAX_BOARDS={max(PORT_MAX_BOARDS.values())}")
    var = await update.new_update(config)
    await cg.register_component(var, config)
    hub = await cg.get_variable(config[CONF_ULTRA_LINE_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_http(await cg.get_variable(config[http_request.CONF_HTTP_REQUEST_ID])))
    cg.add(var.set_source(config.get(CONF_SOURCE, DEFAULT_REPO)))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    keys, count = PUBLIC_KEYS[config[CONF_PUBLIC_KEYS]]
    cg.add(var.set_keys(cg.RawExpression(keys), cg.RawExpression(count)))
    cg.add(var.set_verify_signature(config[CONF_VERIFY_SIGNATURE]))
    for item in config["inventory"]:
        if "boot_identity" in item:
            blob = BOOT_IDENTITIES[item["boot_identity"]]
        else:
            blob = Path(CORE.relative_config_path(item["boot_info"])).read_bytes()
        _check_boot_info(blob)
        uid = bytes.fromhex(item["uid"]) if "uid" in item else b""
        layout = "metadata_base" in item
        blackout = item.get(CONF_BLACKOUT)
        cg.add(var.add_inventory(board_slot(item["board"]), list(blob), list(uid), item["return_channel"], layout,
                                 int(item.get("metadata_base", 0)), item.get("metadata_bytes", 0),
                                 item.get("bootloader_bytes", 0), blackout is not None,
                                 blackout.total_milliseconds if blackout is not None else 0))
    if sel_conf := config.get(CONF_CHANNEL_SELECT):
        sel = await select.new_select(sel_conf, options=list(CHANNELS))
        await cg.register_component(sel, sel_conf)
        cg.add(sel.set_update(var))
    if status_conf := config.get(CONF_STATUS):
        cg.add(var.set_status_sensor(await text_sensor.new_text_sensor(status_conf)))
    if report_conf := config.get(CONF_REPORT):
        cg.add(var.set_report_sensor(await text_sensor.new_text_sensor(report_conf)))


# ------------------------------------------------------------------------------------------------ actions
@automation.register_action(
    "ultra_line.abort_update",
    AbortUpdateAction,
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(UltraFirmwareUpdate)}),
    synchronous=True,
)
async def abort_update_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
