"""dig2analog_plus light: 1-5 channels of a Dig2Analog-Plus-5ch on an ultra_line: board gamma / transitions and host effects.

The schema and the code are the analog lights' (ultra_line/analog_light.py); this platform says the board is
a Dig2Analog-Plus-5ch."""

from esphome.components.ultra_line.analog_light import CONFIG_SCHEMA, to_code

__all__ = ["CONFIG_SCHEMA", "to_code"]  # re-exported: the platform's schema and code

DEPENDENCIES = ["ultra_line"]
