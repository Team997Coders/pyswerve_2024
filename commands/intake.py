import commands2
from wpilib import SmartDashboard
from subsystems import Intake, Indexer


class Outtake(commands2.InstantCommand):
    _intake: Intake

    def __init__(self, intake):
        """
        Pass other subsystems and a logger to this subsystem for debugging

        :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
        :param logger: A python built-in package that handles writing logging messages to netconsole
        """
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._intake = intake

    def execute(self):
        self._intake.velocity = -self._intake.config.default_velocity
        print("Outtake Run")


class IntakeOn(commands2.InstantCommand):
    _intake: Intake

    def __init__(self, intake):
        """Pass other subsystems and a logger to this subsystem for debugging

        :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
        :param logger: A python built-in package that handles writing logging messages to netconsole
        """
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._intake = intake

    def execute(self):
        self._intake.velocity = self._intake.config.default_velocity
        print("Intake On")


class IntakeOff(commands2.InstantCommand):
    _intake: Intake

    def __init__(self, intake):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()
        self._intake = intake

    def execute(self):
        self._intake.velocity = 0
        print("Intake Off")


class Load(commands2.InstantCommand):
    _command: commands2.Command

    def __init__(self, intake: Intake, indexer: Indexer):
        super().__init__()
        self.intake_scheduled = True
        self._command = commands2.cmd.sequence(
            IntakeOn(intake),
            commands2.cmd.waitUntil(lambda: indexer.ready),
            IntakeOff(intake)
        )
        print("intake cunstructed")

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
        print("Intake Execute")
        SmartDashboard.putBoolean("intake scheduled", self.intake_scheduled)
