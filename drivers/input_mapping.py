import math_help
from config import AxisConfig
from typing import Callable


def map_input(read_axis: Callable[[], float], axis_config: AxisConfig) -> float:
    "Convert a controller input value into desired robot scale output values"
    axis_value = read_axis()
    # axis_value = axis_config.input_range.clip(axis_value) # Ensure input falls in expected range.
    return math_help.map_input_to_output_range(axis_value, axis_config.deadband, axis_config.output_range)
