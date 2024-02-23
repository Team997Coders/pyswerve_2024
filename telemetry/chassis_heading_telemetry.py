from subsystems import ChassisHeadingControl, AxisPositionControl
from config import PhysicalConfig
from wpilib import SmartDashboard as sd
import wpilib
from typing import List, Tuple
import math
from telemetry import PIDEntry



class ChassisHeadingTelemetry:
    """Report swerve telemetry to the dashboard.  Compatible with WebComponents"""

    ChassisHeading = ChassisHeadingControl
    pid_entry: PIDEntry

    def __init__(self, chassis_heading_control: ChassisHeadingControl) -> None:
        self.chassis_heading_control = chassis_heading_control
        self.pid_entry = PIDEntry("Chassis Heading PID", chassis_heading_control.pid)

    def report_to_dashboard(self):
        """Write all module info to net tables"""
        self.pid_entry.periodic()
        sd.putNumber("CH: measured rotation", math.degrees(self.chassis_heading_control.getMeasurement()))
        sd.putNumber("CH: target angle for the target tracker", math.degrees(self.chassis_heading_control.target))
        sd.putNumber("CH: feedforward component", self.chassis_heading_control._feedforward_component)
        sd.putNumber("CH: desired velocity", self.chassis_heading_control.desired_velocity)

class AxisPositionTelemetry:
    """Report swerve telemetry to the dashboard.  Compatible with WebComponents"""

    name: str
    axis_heading = AxisPositionControl
    pid_entry: PIDEntry

    def __init__(self, name: str, axis_heading_control: AxisPositionControl) -> None:
        self.name = name
        self.axis_heading = axis_heading_control
        self.pid_entry = PIDEntry(f"{name} position PID", axis_heading_control.pid)

    def report_to_dashboard(self):
        """Write all module info to net tables"""
        self.pid_entry.periodic()

        sd.putNumber(f"{self.name}: measured position",  self.axis_heading.getMeasurement())
        sd.putNumber(f"{self.name}: target position", self.axis_heading.target)
        sd.putNumber(f"{self.name}: feedforward component", self.axis_heading._feedforward_component)
        sd.putNumber(f"{self.name}: desired velocity", self.axis_heading.desired_velocity)
