"""The pixel effect catalogue (pixel_fx_catalogue.json, from quinled-ultra-protocol/catalogue/): the effects and
palettes the QuinLED Diff-Power Ultra-Digital receivers run themselves. ids are never reused (1-127 official, 128-255
custom); the light's effect list and its Palette select are generated from it at compile time.

A byte-identical copy of the protocol repository's file (tests/run_ultra_digital_test.py checks it, like the header)."""

import json
from pathlib import Path

CATALOGUE_FILE = Path(__file__).with_name("pixel_fx_catalogue.json")

PALETTE_DEFAULT = 0xFF   # the effect's own palette
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
        """The Palette select: the effect's own, the three colours, then the catalogue's palettes in id order"""
        return ["Effect default", "Colors 1-2-3"] + [p["name"] for p in sorted(self.palettes, key=lambda p: p["id"])]

    def palette_ids(self) -> list:
        """The palette id of each Palette option (same order as palette_options)"""
        return [PALETTE_DEFAULT, PALETTE_COLORS] + [p["id"] for p in sorted(self.palettes, key=lambda p: p["id"])]


def load(path=None) -> Catalogue:
    with open(path or CATALOGUE_FILE, encoding="utf-8") as f:
        return Catalogue(json.load(f))
