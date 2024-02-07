import unittest
import math_help
from math_help import Range


class controller_input_test(unittest.TestCase):
    def test_deadband_input(self):
        dead_band_end = 0.10
        input_range = Range(dead_band_end, 1)  # Input range is 0 to 1 (absolute)
        output_range = Range(0, 9)  # Robot drives from 0 to 9 m/s
        self.assertEqual(math_help.map_input_to_output_range(0, input_range, output_range), output_range.min_val)
        self.assertEqual(math_help.map_input_to_output_range(1, input_range, output_range), output_range.max_val)
        self.assertAlmostEqual(math_help.map_input_to_output_range(dead_band_end, input_range, output_range),
                               output_range.min_val, places=2)