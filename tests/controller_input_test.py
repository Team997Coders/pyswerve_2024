import unittest
import math_help

class controller_input_test(unittest.TestCase):
    def test_deadband_input(self):
        deadband_end = 0.1
        input_range = math_help.Range(deadband_end, 1)
        output_range = math_help.Range(0, 10)
        self.assertEqual(math_help.map_input_to_output_range(0, input_range,  output_range), output_range.min_val)
        self.assertEqual(math_help.map_input_to_output_range(1, input_range, output_range), output_range.max_val)
        self.assertAlmostEqual(math_help.map_input_to_output_range(deadband_end, input_range, output_range.min_val), 0, places= 2)
        