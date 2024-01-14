import unittest
import math
import math_help

class math_help_test(unittest.TestCase):
    def test_shortest_angle_difference(self):
        self.assertEqual(math_help.shortest_angle_difference(0, 0), 0)
        self.assertEqual(math_help.shortest_angle_difference(0, math.pi / 2), math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(0, -math.pi / 2), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(0, math.pi * 2), 0)
        self.assertEqual(math_help.shortest_angle_difference(0, -math.pi * 2), 0)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, math.pi), math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, -math.pi), math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, math.pi / 2), 0)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, -math.pi / 2), -math.pi)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, math.pi * 2), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(math.pi / 2, -math.pi * 2), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, math.pi), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, -math.pi), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, math.pi / 2), -math.pi)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, -math.pi / 2), 0)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, math.pi * 2), math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(-math.pi / 2, -math.pi * 2), math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(math.pi, math.pi), 0)
        self.assertEqual(math_help.shortest_angle_difference(math.pi, -math.pi), 0)
        self.assertEqual(math_help.shortest_angle_difference(math.pi, math.pi / 2), -math.pi / 2)
        self.assertEqual(math_help.shortest_angle_difference(0, math.pi), -math.pi)