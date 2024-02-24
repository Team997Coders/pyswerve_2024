from wpilib import SmartDashboard as sd
from telemetry import SparkMaxPIDEntry
import rev
import ntcore

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

    sd.putData("Shooter PID", shooter_pid_entry)
    sd.putData("Indexer PID", indexer_pid_entry)
    sd.putData("Intake PID", intake_pid_entry)

def UpdateMechansimPIDs(r):
    sd.putBoolean("Intake Sensor", r.indexer.last_sensor_state)

