"""ur_analog_m light: 1-5 channels of a Diff-Power-UR-Analog-M on an ultra_line: board gamma / transitions and host effects.

The schema and the code are the analog lights' (ultra_line/analog_light.py); this platform says the board is
a Diff-Power-UR-Analog-M."""

from esphome.components.ultra_line.analog_light import CONFIG_SCHEMA, to_code

__all__ = ["CONFIG_SCHEMA", "to_code"]  # re-exported: the platform's schema and code

DEPENDENCIES = ["ultra_line"]
