import unittest
import math
import math_help
from math_help import Range
import robot_config

class controller_input_test(unittest.TestCase):
    def test_deadband_input(self):
        self.assertEqual(math_help.processControllerDeadband(0, Range(robot_config.teleop_controls.y_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed)), 0)
        self.assertEqual(math_help.processControllerDeadband(1, Range(robot_config.teleop_controls.y_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed)), robot_config.physical_properties.max_drive_speed)
        self.assertAlmostEqual(math_help.processControllerDeadband(robot_config.teleop_controls.y_deadband, Range(robot_config.teleop_controls.y_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed)), 0, places= 2)
        