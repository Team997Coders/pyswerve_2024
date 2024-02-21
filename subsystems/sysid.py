import wpilib.sysid
import commands2.sysid
from swerve.swervedrive import SwerveDrive
import robot_config


class swerve_system_id(commands2.subsystem.Subsystem):
    """This class is a wrapper for the wpilib.sysid module."""
    _swerve_drive: SwerveDrive
    _log: wpilib.sysid.SysIdRoutineLog
    _mechanism: commands2.sysid.sysidroutine.SysIdRoutine.Mechanism
    _drive_forward_routine: commands2.sysid.SysIdRoutine

    def __init__(self, swerve_drive: SwerveDrive, log_name: str):
        super().__init__()
        self._swerve_drive = swerve_drive
        self._log = wpilib.sysid.SysIdRoutineLog(log_name)
        self._mechanism = commands2.sysid.sysidroutine.SysIdRoutine.Mechanism(swerve_drive.drive_at_voltage,
                                                                              swerve_drive.log_to_sysid,
                                                                              swerve_drive,
                                                                              "swerve")
        # Config is where to set voltage ramps and voltage steps and other sanity parameters for the motors.
        # starting with the defaults since we don't know what they are yet.
        self._config = commands2.sysid.sysidroutine.SysIdRoutine.Config()

        self._drive_forward_routine = commands2.sysid.SysIdRoutine(self._config, self._mechanism)

    @property
    def log(self) -> wpilib.sysid.SysIdRoutineLog:
        return self._log

    def get_log(self) -> wpilib.sysid.SysIdRoutineLog:
        return self._log

