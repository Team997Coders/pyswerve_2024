from subsystems import ChassisHeadingControl
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
        sd.putNumber("CH: measured rotation", math.degrees(self.chassis_heading_control.getMeasurement()))
        sd.putNumber("CH: target angle for the target tracker", math.degrees(self.chassis_heading_control.target))
        sd.putNumber("CH: feedforward component", self.chassis_heading_control._feedforward_component)
        #sd.putData("CH: pid", self.chassis_heading_control.pid)
        sd.putNumber("CH: desired velocity", self.chassis_heading_control.desired_velocity)
