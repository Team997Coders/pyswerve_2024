import enum
import math
from config import *
  
swerve_modules = {module_position.front_left: 
                    SwerveModuleConfig(drive_motor=MotorConfig(id=8, inverted=False),
                                       turn_motor=MotorConfig(id=1, inverted=True),
                                        encoder=EncoderConfig(id=None, offset=None, conversion_factor=math.pi*2),
                                        location=(12, 12)),
                    module_position.front_right:
                    SwerveModuleConfig(drive_motor=MotorConfig(id=6, inverted=False),
                                       turn_motor=MotorConfig(id=7, inverted=True),
                                        encoder=EncoderConfig(id=None, offset=None, conversion_factor=math.pi*2),
                                        location=(-12, 12)),
                    module_position.back_right:
                    SwerveModuleConfig(drive_motor=MotorConfig(id=4, inverted=False),
                                       turn_motor=MotorConfig(id=5, inverted=True),
                                        encoder=EncoderConfig(id=None, offset=None, conversion_factor=math.pi*2),
                                        location=(-12, -12)),
                    module_position.back_left:
                    SwerveModuleConfig(drive_motor=MotorConfig(id=2, inverted=False),
                                       turn_motor=MotorConfig(id=3, inverted=True),
                                        encoder=EncoderConfig(id=None, offset=None, conversion_factor=math.pi*2),
                                        location=(12, -12))
                    
                 } # type: dict[module_position, SwerveModuleConfig]

physical_properties = PhysicalConfig(wheel_diameter_cm=12,
                                        wheel_grip_coefficient_of_friction=1,
                                        current_limit=OptionalSwerveModuleIntProperty(drive=40, angle=20),
                                        ramp_rate=OptionalSwerveModuleFloatProperty(drive=0.25, angle=0.25),
                                        encoder_pulses_per_revolution=SwerveModuleFloatProperty(drive=1, angle=1),
                                        gear_ratio=SwerveModuleFloatProperty(angle=150.0/7, drive=6.75),
                                        max_drive_speed=1.5)