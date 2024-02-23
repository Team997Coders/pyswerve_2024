from wpilib import SmartDashboard as sd
from telemetry import SparkMaxPIDEntry
import rev

shooter_pid_entry = None
indexer_pid_entry = None
intake_pid_entry = None

def ShowMechansimPIDs(r):
    """Show the PIDs for the mechanisms on the dashboard"""

    #Need to hold globals so references to these objects survive and continue to be updated after the function exits
    global shooter_pid_entry, indexer_pid_entry, intake_pid_entry

    shooter_pid_entry = SparkMaxPIDEntry("Shooter PID", r.shooter.pid, rev.CANSparkMax.ControlType.kVelocity)
    indexer_pid_entry = SparkMaxPIDEntry("Indexer PID", r.indexer.pid, rev.CANSparkMax.ControlType.kVelocity)
    intake_pid_entry = SparkMaxPIDEntry("Intake PID", r.intake.pid, rev.CANSparkMax.ControlType.kVelocity)

def UpdateMechanismPIDs():
    """Update the PIDs for the mechanisms on the dashboard"""
    global shooter_pid_entry, indexer_pid_entry, intake_pid_entry

    if shooter_pid_entry:
        shooter_pid_entry.periodic()
    if indexer_pid_entry:
        indexer_pid_entry.periodic()
    if intake_pid_entry:
        intake_pid_entry.periodic()
