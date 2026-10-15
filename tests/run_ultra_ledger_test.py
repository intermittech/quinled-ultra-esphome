"""Ledger differential tests against frozen a73ff9c (ziglang; output only in tests/build/ledger).

The persistence/line methods are extracted unchanged from production, as in the lifecycle suite.
The compile has 10 min (a cold zig cache, CI), the run 5 min.
"""
from pathlib import Path
import subprocess
import sys
from run_ultra_lifecycle_test import definition

HERE = Path(__file__).resolve().parent
BUILD = HERE / 'build/ledger'

def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    cpp = (HERE.parent / 'components/dig2analog_ultra/dig2analog_ultra.cpp').read_text(encoding='utf-8')
    for filename, signatures in {
        'hub_ledger.inc': ["// The ESP32's one lifetime ledger (every line's", 'void UltraHub::ledger_service_(',
                           'void UltraHub::ledger_save_('],
        'hub_ledger_line.inc': ['void UltraHub::ledger_line_('],
    }.items():
        (BUILD / filename).write_text('\n\n'.join(definition(cpp, s) for s in signatures) + '\n', encoding='utf-8')
    exe = BUILD / ('ultra_ledger_test.exe' if sys.platform == 'win32' else 'ultra_ledger_test')
    subprocess.run([sys.executable, '-m', 'ziglang', 'c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-nullability-completeness', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', f'-I{BUILD}', str(HERE / 'ultra_ledger_test.cpp'),
                    '-o', str(exe)], check=True, timeout=600)
    return subprocess.run([str(exe)], timeout=300).returncode

if __name__ == '__main__':
    sys.exit(main())
