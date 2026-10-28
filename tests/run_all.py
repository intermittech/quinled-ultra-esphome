"""Every host step of CI's host job (.github/workflows/ci.yml, read from there: the two lists can't drift), one at a
time, with ESPHome's Python: python tests/run_all.py [--keep-going]

Needs the protocol and the receiver firmware at ci.yml's pins (tests/receiver_dependency.py, tests/check_pins.py).
ULTRA_TEST_JOBS (default 4) caps the parallel compiles inside a step. Prints "<rc> <seconds>s <step>" per step; exit 1
if any step failed (the first failure stops the run unless --keep-going).
"""
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent


def steps() -> list:
    ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
    host = ci[ci.index("\n  host:"):ci.index("\n  compile:")]
    return re.findall(r"^\s+run: python (tests/\S+\.py)\s*$", host, re.M)


def main() -> int:
    keep = "--keep-going" in sys.argv[1:]
    env = dict(os.environ, ULTRA_TEST_JOBS=os.environ.get("ULTRA_TEST_JOBS", "4"), PYTHONUTF8="1",
               PYTHONIOENCODING="utf-8")
    failed, start = [], time.time()
    for script in steps():
        t0 = time.time()
        rc = subprocess.run([sys.executable, script], cwd=ROOT, env=env).returncode
        print(f"== {rc} {time.time() - t0:.0f}s {script}", flush=True)
        if rc:
            failed.append(script)
            if not keep:
                break
    print(f"== {'FAILED: ' + ', '.join(failed) if failed else 'all passed'} ({time.time() - start:.0f}s)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
