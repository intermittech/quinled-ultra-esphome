"""Power-model entity policy and the light's versioned JSON wire format (no network or board needed)."""
from pathlib import Path
import json
import os
import re
import sys
import subprocess
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "components"))
import esphome.components
esphome.components.__path__.insert(0, str(ROOT / "components"))
from ultra_line.analog_light import output_model_text
from ultra_line.receiver_schema import CORE, RECEIVER_ENTITIES, light_output_model_config, telemetry_defaults


def light(**extra):
    return dict(channel=0, constant_brightness=False, color_interlock=False,
                min_power=dict.fromkeys(["red", "green", "blue", "white1", "white2"], 0),
                max_power=dict.fromkeys(["red", "green", "blue", "white1", "white2"], 1), **extra)


class PowerModelTest(unittest.TestCase):
    def test_digital_json(self):
        work = ROOT / "tests/build/power-model-json"
        work.mkdir(parents=True, exist_ok=True)
        exe = work / ("json.exe" if sys.platform == "win32" else "json")
        subprocess.run([sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O1",
                        "-Wno-nullability-completeness", str(ROOT / "tests/ultra_power_model_json.cpp"),
                        "-o", str(exe)], check=True)
        lines = subprocess.check_output([str(exe)], text=True, encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 6)
        for i, line in enumerate(lines):
            self.assertLessEqual(len(line.encode("ascii")), 255)
            model = json.loads(line)
            self.assertEqual(model["v"], 2)
            self.assertEqual(model["bpp"], 3 + i // 2)
            self.assertEqual(len(model["cp_ma"]), 5)
            if i % 2 == 0:
                self.assertEqual(model["src"], "calibrated")
                self.assertEqual(model["n"], 1)
                self.assertEqual((model["idle_ma"], model["ip_ma"]), (-65536, -65536))
                self.assertEqual(model["cp_ma"][:model["bpp"]], [65535] * model["bpp"])
                self.assertEqual(model["cp_ma"][model["bpp"]:], [0] * (5-model["bpp"]))
            else:
                self.assertEqual((model["src"], model["n"], model["q"]), ("default", 0, 0))
                self.assertEqual(model["cp_ma"], [0] * 5)

    def test_pixel_count_policy(self):
        for level in ("none", "basic", "full"):
            c = telemetry_defaults(dict(num_boards=1, telemetry=dict(diagnostics=level, name="Port 4")))
            r = c["telemetry"]["receivers"][0]
            for key in ("configured_pixel_count", "detected_pixel_count", "pixel_count", "cancel_calibration"):
                self.assertEqual(key in r, level != "none", (key, level))
                if key in r:
                    self.assertFalse(r[key].get("disabled_by_default", False))
            self.assertEqual("used_pixel_count" in r, level == "full")
        for key, slot in (("configured_pixel_count",26), ("detected_pixel_count",27), ("used_pixel_count",28)):
            self.assertEqual(RECEIVER_ENTITIES[key][1], slot)
            validated = RECEIVER_ENTITIES[key][4]({"name":"Unit " + key})
            self.assertEqual(validated["accuracy_decimals"], 0)
            self.assertNotIn("state_class", validated)

    def test_generated_entities(self):
        work = ROOT / "tests/build/power-model"
        work.mkdir(parents=True, exist_ok=True)
        base = f"""esphome:
  name: power-model-test
  devices:
    - id: rx1
      name: Receiver
    - id: lamp
      name: Lamp
esp32:
  board: esp32dev
  framework:
    type: esp-idf
external_components:
  - source:
      type: local
      path: {(ROOT / 'components').as_posix()}
    components: [ultra_line, ur_analog_m]
uart:
  id: rx
  rx_pin: GPIO4
  baud_rate: 1000000
  rx_buffer_size: 1024
ultra_line:
  id: u
  pin: GPIO16
  num_boards: 1
  telemetry:
    uart_id: rx
    receivers: [rx1]
    diagnostics: {{level}}
    light_entities: {{lights}}
light:
  - platform: ur_analog_m
    id: a
    name: A
    board: 1
    channel: 1
{{override}}  - platform: ur_analog_m
    id: b
    name: B
    board: 1
    channel: 2
"""
        for name, level, lights, override, expected in [
            ("full", "full", "true", "", 2), ("basic", "basic", "true", "", 2),
            ("none", "none", "true", "", 0),
            ("override", "none", "true", "    output_model:\n      disabled_by_default: false\n", 1),
            ("device", "full", "true", "    device_id: lamp\n", 2),
            ("device_override", "basic", "true",
             "    device_id: lamp\n    output_model:\n      disabled_by_default: false\n", 2),
            ("no_estimates", "basic", "false", "", 2),
        ]:
            config = work / f"{name}.yaml"
            config.write_text(base.format(level=level, lights=lights, override=override), encoding="utf-8")
            p = subprocess.run([sys.executable, "-m", "esphome", "compile", str(config), "--only-generate"],
                               capture_output=True, text=True, encoding="utf-8", timeout=90,
                               env={**os.environ, "PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8"})
            self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
            cpp = (work / ".esphome/build/power-model-test/src/main.cpp").read_text(encoding="utf-8")
            self.assertEqual(cpp.count('_output_model_holder->set_text('), expected)
            self.assertEqual(cpp.count('"Power model"'), int(level != "none"))
            for label in ("Configured pixel count", "Detected pixel count", "Pixel count", "Cancel calibration"):
                self.assertEqual(cpp.count('"' + label + '"'), int(level != "none"))
            for line in cpp.splitlines():
                if 'App.register_text_sensor(' in line and ('_output_model,' in line or '"Power model"' in line):
                    flags = int(re.search(r', (\d+)\);', line).group(1))
                    self.assertFalse(flags & (1 << 25))   # EntityBase disabled_by_default bit
                    self.assertIn('category:diagnostic', line)
            self.assertEqual('"CH1 full-scale input current"' in cpp, level == "full")
            if expected:
                label = '"Output model"' if name.startswith("device") else '"A output model"'
                self.assertTrue(label in cpp, f"{name}: missing output model name")
                self.assertTrue('\\042channels\\042:[1]' in cpp, f"{name}: missing JSON channel mapping")
            if expected == 2:
                self.assertTrue('"B output model"' in cpp, f"{name}: missing second light's model")

    def test_output_model_policy_and_names(self):
        c = dict(id="strip", name="Strip", board=1, ultra_line_id="hub")   # (a validated config: board 1 = the first)
        h = dict(id="hub", telemetry=dict(diagnostics="full", receivers=[{}], light_entities=True))
        with patch.object(CORE, "config", dict(ultra_line=[h], light=[c])):
            ent = light_output_model_config(c)
            self.assertEqual(ent, dict(id="strip_output_model", name="Strip output model"))
            c["device_id"] = "rx"
            self.assertEqual(light_output_model_config(c)["name"], "Output model")
            CORE.config["light"].append(dict(device_id="rx"))
            self.assertEqual(light_output_model_config(c)["name"], "Strip output model")
            h["telemetry"]["diagnostics"] = "basic"
            self.assertIsNotNone(light_output_model_config(c))
            for level in ("none",):
                h["telemetry"]["diagnostics"] = level
                self.assertIsNone(light_output_model_config(c))
            c["output_model"] = dict(name="My model", disabled_by_default=False)
            self.assertEqual(light_output_model_config(c)["name"], "My model")
            self.assertFalse(light_output_model_config(c)["disabled_by_default"])
            h["telemetry"]["light_entities"] = False
            self.assertIsNotNone(light_output_model_config(c))
            h["telemetry"] = {}
            self.assertIsNone(light_output_model_config(c))

    def test_mono_and_gamma(self):
        for gamma, expected in [(0, 1), (1, 1), (2.8126, 2.813), (100, 65), (.00001, .001)]:
            m = json.loads(output_model_text(light(gamma_correct=gamma)))
            self.assertEqual(m["channels"], [1])
            self.assertEqual(m["gamma_correct"], expected)
            self.assertEqual((m["min_mireds"], m["max_mireds"]), (0, 0))

    def test_all_channel_layouts_and_text_bound(self):
        for roles in [("channel",), ("white",), ("red", "green", "blue"),
                      ("red", "green", "blue", "white"), ("cold_white", "warm_white"),
                      ("red", "green", "blue", "cold_white", "warm_white")]:
            c = light()
            del c["channel"]
            c.update({r: 4-i for i, r in enumerate(roles)})
            c.update(constant_brightness=True, color_interlock=True,
                     cold_white_color_temperature=153.846153846, warm_white_color_temperature=555.555555555)
            text = output_model_text(c)
            m = json.loads(text)
            self.assertEqual(m["channels"], [5-i for i in range(len(roles))])
            self.assertEqual(m["v"], 1)
            self.assertTrue(m["constant_brightness"] and m["color_interlock"])
            self.assertAlmostEqual(m["min_mireds"], 153.846, places=3)
            self.assertEqual(m["native_type"], "ultra")
            self.assertFalse(m["nonlinear"])
            self.assertLessEqual(len(text.encode("ascii")), 255)

    def test_nonlinear(self):
        for setting in ("dim_to_warm", "min_power", "max_power"):
            c = light()
            if setting == "dim_to_warm":
                c[setting] = {}
            else:
                c[setting]["red"] = .5
            self.assertTrue(json.loads(output_model_text(c))["nonlinear"])

    def test_entity_levels_and_stable_names(self):
        keys = [f"ch{i}_full_scale_input_current" for i in range(1, 6)] + ["idle_input_current"]
        for level in ("none", "basic", "full"):
            c = telemetry_defaults(dict(num_boards=1, telemetry=dict(diagnostics=level, name="Port 4")))
            r = c["telemetry"]["receivers"][0]
            self.assertEqual("power_model" in r, level != "none")
            if level != "none":
                self.assertFalse(r["power_model"].get("disabled_by_default", False))
                self.assertEqual(r["power_model"]["name"], "Port 4 R1 power model")
            for i, k in enumerate(keys):
                self.assertEqual(k in r, level == "full")
                self.assertEqual(RECEIVER_ENTITIES[k][1:3], (20+i, 2))
                if k in r:
                    self.assertTrue(r[k]["disabled_by_default"])
                    suffix = f"CH{i+1} full-scale input current" if i < 5 else "idle input current"
                    self.assertEqual(r[k]["name"], "Port 4 R1 " + suffix)
        c = telemetry_defaults(dict(telemetry=dict(diagnostics="basic", receivers=[
            dict(device_id="rx", ch1_full_scale_input_current=dict(name="My coefficient", disabled_by_default=False))])))
        r = c["telemetry"]["receivers"][0][keys[0]]
        self.assertEqual(r, dict(name="My coefficient", disabled_by_default=False, device_id="rx"))


if __name__ == "__main__":
    unittest.main()
