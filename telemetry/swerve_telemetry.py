from swerve import SwerveDrive
from config import PhysicalConfig
from wpilib import SmartDashboard as sd
from typing import List, Tuple 

class SwerveTelemetry:
    """Report swerve telemetry to the dashboard.  Compatible with WebComponents"""

    swerve_drive: SwerveDrive
    physical_config: PhysicalConfig

    def __init__(self, swerve_drive: SwerveDrive, physical_config: PhysicalConfig,) -> None:
        self.swerve_drive = swerve_drive
        self.physical_config = physical_config
        self.report_static_to_dashboard()

    @staticmethod
    def unpack_tuples(tuples: List[Tuple[float, float]]) -> list[float]:
        """Unpack list of tuples into a list of all tuple values"""
        return [i for item in tuples for i in item]
    
    def report_static_to_dashboard(self):
        """Write unchanging values to dashboard"""
        sd.putNumber("swerve/moduleCount", self.swerve_drive.num_modules)
        sd.putNumberArray("swerve/wheelLocations", self.unpack_tuples([(m.location.X(), m.location.Y()) for m in self.swerve_drive.ordered_modules]))
        sd.putNumber("swerve/maxSpeed", self.physical_config.max_drive_speed)

        locations = [m.location for m  in self.swerve_drive.ordered_modules]
        minx = min([l.X() for l in locations])
        maxx = max([l.X() for l in locations])
        miny = min([l.Y() for l in locations])
        maxy = max([l.Y() for l in locations])
        sd.putNumber("swerve/sizeLeftRight", maxy - miny)
        sd.putNumber("swerve/sizeFrontBack", maxx - minx)
        sd.putString("swerve/rotationUnit", "degrees")

        # for m in self.swerve_drive.ordered_modules:
        #     sd.putData(f"swerve/{m.position}/angle_pid", m.angle_pid)
        #     sd.putData(f"swerve/{m.position}/drive_pid", m.drive_pid)
    
    def report_to_dashboard(self):
        """Write all module info to nettables"""
        sd.putNumberArray("swerve/measuredStates", self.unpack_tuples(
            [(m.measured_state.angle.degrees(), m.measured_state.speed) for m in self.swerve_drive.ordered_modules]))
        sd.putNumberArray("swerve/desiredStates", self.unpack_tuples([(m.desired_state.angle.degrees(),
                                                                       m.desired_state.speed)
                                                                      for m in self.swerve_drive.ordered_modules]))
        sd.putNumber("swerve/robotRotation", self.swerve_drive.gyro_angle_degrees)
        measured_chassis_speed = self.swerve_drive.measured_chassis_speed
        sd.putNumberArray("swerve/measuredChassisSpeeds", [measured_chassis_speed.vx,
                                                           measured_chassis_speed.vy,
                                                           measured_chassis_speed.omega])
         
        #sd.putString("swerve/forwardDirection", forwardDirection)
        #sd.putNumber("swerve/maxAngularVelocity", maxAngularVelocity)
        # sd.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds)
        desired_chassis_speeds = self.swerve_drive.desired_chassis_speed
        sd.putNumberArray("swerve/desiredChassisSpeeds", desired_chassis_speeds)