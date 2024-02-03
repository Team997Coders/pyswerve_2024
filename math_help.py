import math

from math_helper import Range
import numpy as np
import wpimath.geometry as geom
import wpimath.kinematics as kinematics

def wrap_angle(angle: float, min: float = 0) -> float:
        '''Wrap the angle to the range of 0 to 2pi'''
        clamped = angle % (math.pi * 2.0)
        if min != 0:
              max = min + math.pi * 2.0
              while clamped > max: # TODO Do the math and multiply instead of adding
                  clamped -= math.pi * 2.0
              while clamped < min:
                  clamped += math.pi * 2.0
        return clamped


def shortest_angle_difference( angle1: float, angle2: float) -> float:
    '''Returns the shortest angle difference between two angles'''
    diff = (angle2 - angle1 + math.pi) % (math.pi * 2) - math.pi
    return diff

def optimize_state_improved(desired_state: kinematics.SwerveModuleState, current_angle: geom.Rotation2d) -> kinematics.SwerveModuleState:
    '''Returns the optimized angle for a swerve module.  This is a modified version of the wpilib implementation that uses
       the dot product to determine angle reversal and scales the output speed according to the dot product as well.'''
    desired_angle = desired_state.angle
    desired_vector = np.array([desired_angle.cos(), desired_angle.sin()])
    current_vector = np.array([current_angle.cos(), current_angle.sin()])
    result = np.vdot(desired_vector, current_vector) # type: float # type: ignore
    if __debug__:
        assert(-1.00001 <= result <= 1.00001) # Use an espsilon to account for floating point errors

    desired_speed = desired_state.speed * result
    
    if np.isclose(result, 0): # If the dot product is 0, we don't need to change the angle
        desired_speed = 0
        desired_state = kinematics.SwerveModuleState(desired_speed, desired_angle)
    elif result < 0: # If the dot product is negative, we need to reverse the desired angle, and scale speed by the dot product
        desired_angle = desired_angle + geom.Rotation2d(math.pi) 
        desired_state = kinematics.SwerveModuleState(desired_speed, desired_angle)
    else: # No change needed to the desired angle
        desired_state = kinematics.SwerveModuleState(desired_speed, desired_angle)

    return desired_state


def optimize_improved(desired_angle_radians: float, desired_speed: float, current_angle_radians: float) -> kinematics.SwerveModuleState:
    '''Returns the optimized angle for a swerve module'''
    desired_state = kinematics.SwerveModuleState(desired_speed, geom.Rotation2d(desired_angle_radians))
    current_rotation = geom.Rotation2d(current_angle_radians)
    return optimize_state_improved(desired_state, current_rotation)

def processControllerDeadband(input, input_range:Range, output_range:Range):
    '''proccesses stick input from a controller so that '''
    input_norm = input_range.normalize(abs(input))
    # input_one_norm = self.inputControllerOneRange.normalize(abs(input))
    if input_norm > 0:
        input_adjusted = output_range.interpolate(input_norm)
        input_adjusted = input_adjusted if input >= 0 else -input_adjusted
    else:
        input_adjusted = 0

    return input_adjusted