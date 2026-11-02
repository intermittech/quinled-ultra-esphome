"""Check board HA groups, palette data and V203 protocol parity without ESPHome or a C++ build."""
import json
import os
from collections import Counter
from pathlib import Path
from runpy import run_path

from receiver_dependency import protocol_dir

ROOT = Path(__file__).resolve().parents[1]
ORDER = ['Calm', 'Nature', 'Colour', 'Sparkle', 'Fire & candle', 'Motion', 'Party', 'Seasonal', 'Other']
COUNTS = {'v203': [12, 0, 12, 6, 3, 0, 0, 3, 0], 'v407': [23, 22, 14, 6, 4, 21, 13, 11, 0]}


def without_ha_groups(data):
    return {key: [{k: v for k, v in effect.items() if k != 'ha_group'} for effect in value]
            if key == 'effects' else value for key, value in data.items() if key != 'ha_group_order'}


def check_groups():
    cats = {}
    for board, filename in [('v203', 'pixel_fx_catalogue.json'), ('v407', 'pixel_fx_catalogue_v407.json')]:
        cat = json.loads((ROOT / 'components/ur_digital_m' / filename).read_text(encoding='utf-8'))
        cats[board] = cat
        assert cat['ha_group_order'] == ORDER, board
        effects = [e for e in cat['effects'] if not e.get('retired')]
        assert all(e.get('ha_group') in ORDER[:-1] for e in effects), f'{board}: unassigned or Other'
        counts = Counter(e['ha_group'] for e in effects)
        assert [counts[g] for g in ORDER] == COUNTS[board], (board, counts)
        print(f'PASS {board}: {len(effects)} effects; {dict(counts)}; Other=0')
    assert len(cats['v203']['palettes']) == 31
    palettes = cats['v407']['palettes']
    assert [p['id'] for p in palettes] == list(range(49))
    assert palettes[:31] == cats['v203']['palettes']
    assert all(len(p['stops']) == 16 for p in palettes[31:])
    assert [p['name'] for p in palettes[43:]] == ['Fig and Brass', 'Mulberry Milk', 'Clay and Linen', 'Neon Noir', 'Tide Pool', 'Graphite Citrus']
    resolved = [e['design'] for e in cats['v407']['effects'] if 'default_palette_id' in e.get('design', {})]
    assert len(resolved) == 17
    names = {p['id']: p['name'] for p in palettes}
    assert all(names[d['default_palette_id']] == d['default_palette_name'] for d in resolved)
    print('PASS V407: 49 palettes, 16-stop swatches and 17 resolved effect defaults; V203: 31 unchanged palettes')
    assert cats['v407']['effects'][:36] == cats['v203']['effects'], 'shared entries and groups differ'
    protocol = protocol_dir()
    reference = protocol / 'catalogue/pixel_fx_catalogue.json'
    if not reference.exists():
        raise FileNotFoundError(f'{reference} (the protocol checkout; ULTRA_PROTOCOL)')
    if True:
        assert without_ha_groups(cats['v203']) == without_ha_groups(json.loads(reference.read_text(encoding='utf-8'))), 'V203 protocol data changed'
        errors = run_path(str(protocol / 'catalogue/check_catalogue.py'))['check'](without_ha_groups(cats['v203']))
        assert not errors, errors
        print('PASS V203 protocol data unchanged apart from HA groups')


if __name__ == '__main__':
    check_groups()
