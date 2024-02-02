from swerve import SwerveDrive
from config import PhysicalConfig
from wpilib import SmartDashboard as sd
from typing import List, Tuple
import math

class SwerveTelemetry():
    '''Report swerve telemetry to the dashboard.  Compatible with WebComponents'''

    swerve_drive: SwerveDrive
    physical_config: PhysicalConfig

    def __init__(self, swerve_drive: SwerveDrive, physical_config: PhysicalConfig,) -> None:
        self.swerve_drive = swerve_drive
        self.physical_config = physical_config

    @staticmethod
    def unpack_tuples(tuples: List[Tuple[float, float]]) -> list[float]:
        '''Unpack list of tuples into a list of all tuple values'''
        return [i for item in tuples for i in item]
    
    def report_to_dashboard(self):
        '''Write all module info to nettables''' 
        sd.putNumber("swerve/moduleCount", self.swerve_drive.num_modules)
        sd.putNumberArray("swerve/wheelLocations", self.unpack_tuples([(m.location.X(), m.location.Y()) for m in self.swerve_drive.ordered_modules]))
        sd.putNumberArray("swerve/measuredStates", self.unpack_tuples([(m.measured_state.angle.degrees(), m.measured_state.speed) for m in self.swerve_drive.ordered_modules]))
        sd.putNumberArray("swerve/desiredStates", self.unpack_tuples([(m.desired_state.angle.degrees(), m.desired_state.speed) for m in self.swerve_drive.ordered_modules]))
        sd.putNumber("swerve/robotRotation", 0)
        sd.putNumber("swerve/maxSpeed", self.physical_config.max_drive_speed)
        sd.putString("swerve/rotationUnit", "degrees")
        sd.putNumber("swerve/measuredChassisSpeeds", math.sqrt(self.swerve_drive.measured_chassis_speed.vx **2 + self.swerve_drive.measured_chassis_speed.vy **2))
        #sd.putNumber("swerve/sizeLeftRight", sizeLeftRight)
        #sd.putNumber("swerve/sizeFrontBack", sizeFrontBack)
        #sd.putString("swerve/forwardDirection", forwardDirection)
        #sd.putNumber("swerve/maxAngularVelocity", maxAngularVelocity)
        # sd.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds)
        #sd.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds)

