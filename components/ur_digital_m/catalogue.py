"""The pixel effect catalogue (pixel_fx_catalogue.json, from quinled-ultra-protocol/catalogue/): the effects and
palettes the Diff-Power Ultra Receiver Digital-Medium boards run themselves. ids are never reused (1-127 official, 128-255
custom); the light's effect list and its Palette select are generated from it at compile time.

The V203 file matches the protocol catalogue with additional HA group metadata. The V407 file uses the same
format with the approved 114 effects / 49 palettes; design metadata and RGB palette stops are retained.
tests/run_ur_digital_m_test.py checks the shared entries and the V203 protocol copy."""

import json
from pathlib import Path

CATALOGUE_FILE = Path(__file__).with_name("pixel_fx_catalogue.json")
BOARD_CATALOGUES = {"v203": CATALOGUE_FILE, "v407": CATALOGUE_FILE.with_name("pixel_fx_catalogue_v407.json")}

PALETTE_DEFAULT = 0xFF   # Effect default: the effect's default palette or its own colours
PALETTE_COLORS = 0xFE    # c1 -> c2 -> c3
SOLID_NAME = "solid"


class CatalogueError(ValueError):
    pass


class Catalogue:
    def __init__(self, data: dict):
        self.version = data.get("catalogue_version")
        if not isinstance(self.version, int):
            raise CatalogueError("catalogue_version must be an integer")
        self.effects = [e for e in data.get("effects", []) if not e.get("retired")]
        self.palettes = [p for p in data.get("palettes", []) if not p.get("retired")]
        for kind, items in (("effect", self.effects), ("palette", self.palettes)):
            if not items:
                raise CatalogueError(f"the catalogue has no {kind}s")
            ids = [i["id"] for i in items]
            names = [i["name"].lower() for i in items]
            if len(set(ids)) != len(ids) or len(set(names)) != len(names):
                raise CatalogueError(f"{kind} ids and names must be unique")
        self._by_id = {e["id"]: e for e in self.effects}

    def effect(self, effect_id: int) -> dict:
        return self._by_id[effect_id]

    def effect_by_name(self, name: str):
        for e in self.effects:
            if e["name"].lower() == name.lower():
                return e
        return None

    @property
    def solid_id(self) -> int:
        """The effect a light shows without an effect chosen (its colour, with smooth transitions): 'Solid'"""
        e = self.effect_by_name(SOLID_NAME)
        if e is None:
            raise CatalogueError("the catalogue has no 'Solid' effect")
        return e["id"]

    def palette_options(self) -> list:
        """The Palette select: Effect default, the three colours, then the catalogue's palettes in id order"""
        return ["Effect default", "Colors 1-2-3"] + [p["name"] for p in sorted(self.palettes, key=lambda p: p["id"])]

    def palette_ids(self) -> list:
        """The palette id of each Palette option (same order as palette_options)"""
        return [PALETTE_DEFAULT, PALETTE_COLORS] + [p["id"] for p in sorted(self.palettes, key=lambda p: p["id"])]


def load(path=None, board_type="v203") -> Catalogue:
    with open(path or BOARD_CATALOGUES[board_type], encoding="utf-8") as f:
        return Catalogue(json.load(f))


def capabilities(board_type):
    """Enabled diagnostic text: hex bytes, bit (id & 7) of byte (id >> 3), as page 0x11.

    192 bits per map keeps the full JSON below HA's 255-character state limit.
    """
    cat = load(board_type=board_type)
    def bitmap(items):
        data = bytearray(24)
        for item in items:
            data[item["id"] >> 3] |= 1 << (item["id"] & 7)
        return data.hex()
    return json.dumps({"v": 1, "source": "build", "board": board_type,
                       "effects": bitmap(cat.effects), "palettes": bitmap(cat.palettes)}, separators=(",", ":"))
