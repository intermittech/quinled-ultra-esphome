"""Copy examples to tests/build/examples-local with the GitHub external_components source replaced by this repository's
local components folder, so a compile tests the working tree instead of what is on GitHub.

Usage: .venv/Scripts/python tests/local_examples.py [example.yaml ...]   (default: every example)
then (PowerShell, ESP-IDF refuses Git Bash):
  & .\\.venv\\Scripts\\esphome.exe compile tests\\build\\examples-local\\<example>.yaml
"""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SRC = ROOT / "examples"
OUT = HERE / "build" / "examples-local"
GH = "source: github://intermittech/quinled-ultra-esphome@main"


def main() -> int:
    names = sys.argv[1:] or sorted(p.name for p in SRC.glob("quinled-*.yaml"))
    OUT.mkdir(parents=True, exist_ok=True)
    comp = (ROOT / "components").as_posix()
    for n in names:
        out, hits = [], 0
        for ln in (SRC / n).read_text(encoding="utf-8").split("\n"):
            if ln.strip() == "- " + GH:
                ind = ln[: len(ln) - len(ln.lstrip())]
                out += [ind + "- source:", ind + "    type: local", ind + "    path: " + comp]
                hits += 1
            else:
                out.append(ln)
        if hits != 1:
            print(f"{n}: expected one '{GH}', found {hits}")
            return 1
        (OUT / n).write_text("\n".join(out), encoding="utf-8", newline="\n")
    secrets = SRC / "secrets.yaml"
    (OUT / "secrets.yaml").write_text(
        secrets.read_text(encoding="utf-8") if secrets.exists()
        else (
            'wifi_ssid: "test"\nwifi_password: "testtest"\n'
            'api_encryption_key: "AQIDBAUGBwgJCgsMDQ4PEBESExQVFhcYGRobHB0eHyA="\n'
            'ota_password: "testtest"\nfallback_ap_password: "testtest"\n'
        ),
        encoding="utf-8",
    )
    print(f"{len(names)} example(s) in {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
