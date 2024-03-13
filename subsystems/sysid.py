import wpilib.sysid
import commands2.sysid
import commands2
from swerve.swervedrive import SwerveDrive
from commands.drive import SetSwerveModuleAngles

class swerve_system_id(commands2.subsystem.Subsystem):
    """This class is a wrapper for the wpilib.sysid module."""
    _swerve_drive: SwerveDrive
    _log: wpilib.sysid.SysIdRoutineLog
    _mechanism: commands2.sysid.sysidroutine.SysIdRoutine.Mechanism
    drive_forward_routine: commands2.sysid.SysIdRoutine
    drive_reverse_routine: commands2.sysid.SysIdRoutine

    def create_quasistatic_measurement_command(self) -> commands2.Command:
        """Creates a command that aligns the wheels and performs a series of kinetic measurements."""
        return commands2.SequentialCommandGroup(
            SetSwerveModuleAngles(self._swerve_drive, angle=0),
            commands2.cmd.WaitCommand(1),
            self.drive_forward_routine.quasistatic(direction=commands2.sysid.SysIdRoutine.Direction.kReverse),
            self.drive_forward_routine.quasistatic(direction=commands2.sysid.SysIdRoutine.Direction.kForward),
            self.drive_forward_routine.quasistatic(direction=commands2.sysid.SysIdRoutine.Direction.kReverse),
            self.drive_forward_routine.quasistatic(direction=commands2.sysid.SysIdRoutine.Direction.kForward),
        )

    def create_dynamic_measurement_command(self) -> commands2.Command:
        """Creates a command that aligns the wheels and performs a series of kinetic measurements."""
        return commands2.SequentialCommandGroup(
            SetSwerveModuleAngles(self._swerve_drive, angle=0),
            commands2.cmd.WaitCommand(1),
            self.drive_forward_routine.dynamic(direction=commands2.sysid.SysIdRoutine.Direction.kReverse),
            self.drive_forward_routine.dynamic(direction=commands2.sysid.SysIdRoutine.Direction.kForward),
            self.drive_forward_routine.dynamic(direction=commands2.sysid.SysIdRoutine.Direction.kReverse),
            self.drive_forward_routine.dynamic(direction=commands2.sysid.SysIdRoutine.Direction.kForward),
        )

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
        self._config = commands2.sysid.sysidroutine.SysIdRoutine.Config(stepVoltage=3)

        self.drive_forward_routine = commands2.sysid.SysIdRoutine(self._config, self._mechanism)

    @property
    def log(self) -> wpilib.sysid.SysIdRoutineLog:
        return self._log

    def get_log(self) -> wpilib.sysid.SysIdRoutineLog:
        return self._log


