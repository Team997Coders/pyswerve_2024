from subsystems import ChassisHeadingControl, chassis_heading_control
from config import PhysicalConfig
from wpilib import SmartDashboard as sd
from typing import List, Tuple
import math


class ChassisHeadingTelemetry:
    """Report swerve telemetry to the dashboard.  Compatible with WebComponents"""

    ChassisHeading = ChassisHeadingControl

    def __init__(self, chassis_heading_control: ChassisHeadingControl) -> None:
        self.chassis_heading_control = chassis_heading_control

    def report_to_dashboard(self):
        """Write all module info to net tables"""
        sd.putNumber("measured rotation", math.degrees(self.chassis_heading_control.getMeasurement()))
        sd.putNumber("target angle for the target tracker", math.degrees(self.chassis_heading_control.target))
        sd.putNumber("feedforward component", self.chassis_heading_control._feedforward_component)
        sd.putNumber("pid component", self.chassis_heading_control._pid_component)
        sd.putNumber("desired velocity", self.chassis_heading_control.desired_velocity)
