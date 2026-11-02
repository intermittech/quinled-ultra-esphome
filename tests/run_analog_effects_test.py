"""Run production analog effects and the light's output path with small ESPHome stand-ins."""
from pathlib import Path
import hashlib
import subprocess
import sys
from run_ultra_lifecycle_test import definition

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BUILD = HERE / 'build' / 'analog_effects'

def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    comp = ROOT / 'components' / 'ultra_line'
    effects = (comp / 'ultra_effects.h').read_text()
    effects = '\n'.join(line for line in effects.splitlines() if not line.startswith(('#include', '#pragma', '#ifdef', '#endif')))
    (BUILD / 'effects.inc').write_text(effects)
    methods = (comp / 'ultra_light.cpp').read_text()
    signatures = ['void UltraLight::send_(', 'void UltraLight::loop(', 'uint32_t UltraLight::effect_interval(',
                  'void UltraLight::send_pixel(', 'void UltraLight::send_stored_level_(']
    (BUILD / 'output.inc').write_text('\n'.join(definition(methods, sig) for sig in signatures))
    native = Path(__import__('esphome').__file__).parent / 'components' / 'light' / 'base_light_effects.h'
    text = native.read_text()
    names = ['inline static float random_cubic_float(', 'class PulseLightEffect ', 'class RandomLightEffect ',
             'struct StrobeLightEffectColor ', 'class StrobeLightEffect ', 'class FlickerLightEffect ']
    (BUILD / 'native.inc').write_text('\n'.join(definition(text, sig) + (';' if sig.startswith(('class', 'struct')) else '') for sig in names))
    exe = BUILD / ('analog_effects_test.exe' if sys.platform == 'win32' else 'analog_effects_test')
    subprocess.run([sys.executable, '-m', 'ziglang', 'c++', '-std=c++17', '-O1', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', '-Wno-nullability-completeness', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', f'-I{BUILD}', str(HERE / 'analog_effects_test.cpp'), '-o', str(exe)], check=True)
    samples = BUILD / 'samples.bin'
    rc = subprocess.run([str(exe), str(samples)]).returncode
    if rc:
        return rc
    contract = BUILD / ('analog_v03_contract_test.exe' if sys.platform == 'win32' else 'analog_v03_contract_test')
    subprocess.run([sys.executable, '-m', 'ziglang', 'c++', '-std=c++17', '-O1', '-Wall', '-Wextra',
                    '-Werror', '-Wno-nullability-completeness', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', str(HERE / 'analog_v03_contract_test.cpp'), '-o', str(contract)], check=True)
    return subprocess.run([str(contract), str(samples)]).returncode

if __name__ == '__main__':
    sys.exit(main())
