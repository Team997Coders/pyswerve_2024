import unittest
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
import hypothesis.strategies
import math_help
import math

AngleZero = geom.Rotation2d(0)
AngleFortyFive = geom.Rotation2d(45)
AngleNinety = geom.Rotation2d(90)
AngleNegFortyFive = geom.Rotation2d(-45)
AngleNegNinety = geom.Rotation2d(-90)

class TestSwerveModuleState(unittest.TestCase):

    @staticmethod
    def is_angle_difference_greater_than_180(old_angle: float, new_angle: float) -> float:
        return abs(old_angle - new_angle) > 180

    def test_optimize_trivial(self):
        # Test case 1: Zero speed and zero angle
        module_state = kinematics.SwerveModuleState(0, geom.Rotation2d(0))
        optimized_state = module_state.optimize(module_state, geom.Rotation2d(0))
        self.assertEqual(optimized_state.speed, 0)
        self.assertEqual(optimized_state.angle, AngleZero)

        # Test case 2: Non-zero speed and zero angle
        module_state = kinematics.SwerveModuleState(0.5, geom.Rotation2d(0))
        optimized_state = module_state.optimize(module_state, geom.Rotation2d(0))
        self.assertEqual(optimized_state.speed, 0.5)
        self.assertEqual(optimized_state.angle, AngleZero)

        # Test case 3: Zero speed and non-zero angle
        angle = geom.Rotation2d(45)
        module_state = kinematics.SwerveModuleState(0, angle)
        optimized_state = module_state.optimize(module_state, angle)
        self.assertEqual(optimized_state.speed, 0)
        self.assertEqual(optimized_state.angle, angle)

        # # Test case 4: Non-zero speed and non-zero angle
        angle = geom.Rotation2d(90)
        module_state = kinematics.SwerveModuleState(0.75, angle)
        optimized_state = module_state.optimize(module_state, angle)
        self.assertEqual(optimized_state.speed, 0.75)
        self.assertEqual(optimized_state.angle, angle)

    @hypothesis.given(currentRadians=hypothesis.strategies.floats(min_value=0, max_value=2*math.pi),
                      desiredRadians=hypothesis.strategies.floats(min_value=0, max_value=2*math.pi),
                      speed=hypothesis.strategies.floats(min_value=0, max_value=1))
    @hypothesis.settings(max_examples=50, deadline=10000)
    def test_optimization(self, currentRadians: float, desiredRadians: float, speed: float):
        """These are unlikely to fail, but are good to have for regression testing.  Comment them if they delay deployment"""
        self.check_optimization(currentRadians, desiredRadians, speed)

    def check_optimization(self, currentRadians: float, desiredRadians: float, speed: float):
        currentAngle = geom.Rotation2d(currentRadians)
        desiredAngle = geom.Rotation2d(desiredRadians)
        difference = math_help.shortest_angle_difference(currentRadians, desiredRadians)
        module_state = kinematics.SwerveModuleState(speed, desiredAngle)
        optimized_state = kinematics.SwerveModuleState.optimize(module_state, currentAngle)

        if abs(difference) < math.pi / 2.0: #If the turn is less than 90 degrees drive direction should be the same and angle should match
            self.assertEqual(optimized_state.speed, speed)
            self.assertAlmostEqual(optimized_state.angle.radians(), desiredRadians)
        else: #Otherwise, drive direction is reversed and angle should be 180 degrees from desired
            self.assertEqual(optimized_state.speed, -speed)
            angle_diff = optimized_state.angle.radians() - math_help.wrap_angle(desiredRadians, min_val=-math.pi)
            self.assertAlmostEqual(abs(angle_diff), math.pi)

    def test_falsifying_case(self):
        """Use this test case to reproduce hypothesis testing failures"""
        self.check_optimization(0, (math.pi * 3) / 2, 1)

    @hypothesis.given(currentRadians=hypothesis.strategies.floats(min_value=0, max_value=2*math.pi),
                      desiredRadians=hypothesis.strategies.floats(min_value=0, max_value=2*math.pi),
                      speed=hypothesis.strategies.floats(min_value=0, max_value=1))
    @hypothesis.settings(max_examples=50, deadline=10000)
    def test_improved_optimization(self, currentRadians: float, desiredRadians: float, speed: float):
        """These are unlikely to fail, but are good to have for regression testing.  Comment them if they delay deployment"""
        self.check_improved_optimization(currentRadians, desiredRadians, speed)

    def check_improved_optimization(self, currentRadians: float, desiredRadians: float, speed: float):
        currentAngle = geom.Rotation2d(currentRadians)
        desiredAngle = geom.Rotation2d(desiredRadians)
        difference = math_help.shortest_angle_difference(currentRadians, desiredRadians)
        module_state = kinematics.SwerveModuleState(speed, desiredAngle)
        optimized_state = math_help.optimize_state_improved(module_state, currentAngle)

        if abs(difference) <= math.pi / 2.0: #If the turn is less than 90 degrees drive direction should be the same and angle should match
            
            self.assertEqual(optimized_state.speed >= 0, speed >= 0) # check for same sign
            self.assertAlmostEqual(optimized_state.angle.radians(), desiredRadians)
        else: #Otherwise, drive direction is reversed and angle should be 180 degrees from desired
            if optimized_state.speed > 0:
                self.assertEqual(optimized_state.speed >= 0 , -speed >= 0) # Check for opposite signs

            angle_diff = optimized_state.angle.radians() - math_help.wrap_angle(desiredRadians, min_val=-math.pi)
            self.assertAlmostEqual(abs(angle_diff), math.pi)
    
    def test_falsifying_improved_case(self):
        """Use this test case to reproduce hypothesis testing failures"""
        self.check_improved_optimization(6.210083064008053,
                                         6.210083064008053, 2)

if __name__ == '__main__':
    unittest.main()