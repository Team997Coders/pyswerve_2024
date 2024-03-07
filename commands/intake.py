import commands2
from wpilib import SmartDashboard
from subsystems import Intake, Indexer
from commands import IndexOnIntake, IndexOff


class Outtake(commands2.InstantCommand):
    _intake: Intake
    _index: Indexer

    def __init__(self, intake, index):
        """
        Pass other subsystems and a logger to this subsystem for debugging

        :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
        :param logger: A python built-in package that handles writing logging messages to netconsole
        """
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._intake = intake
        self._index = index

    def execute(self):
        # self._intake.velocity = -self._intake.config.default_velocity
        self._intake.speed = self._intake.config.outtake_velocity
        self._index.speed = self._index.config.outtake_velocity
        print("Outtake Run")


class IntakeOn(commands2.InstantCommand):
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
        # self._intake.velocity = 1  # self._intake.config.default_velocity
        self._intake.speed = self._intake.config.intake_velocity
        print("Intake On")


class IntakeOff(commands2.InstantCommand):
    _intake: Intake

    def __init__(self, intake):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()
        self._intake = intake

    def execute(self):
        self._intake.speed = 0
        print("Intake Off")


class IndexSensorCommand(commands2.WaitUntilCommand):
    _indexer: Indexer

    def __init__(self, indexer):
        self._indexer = indexer
        super().__init__(lambda: self._indexer.ready)


class Load(commands2.InstantCommand):
    """Loads a note (ring) into the robot and prepares it to be fired"""
    _command: commands2.Command

    def __init__(self, intake: Intake, indexer: Indexer):
        super().__init__()
        self.intake_scheduled = True
        self._command = commands2.SequentialCommandGroup(
            commands2.cmd.ParallelCommandGroup(IntakeOn(intake),
                                               IndexOnIntake(indexer)),
            commands2.cmd.ParallelRaceGroup(commands2.WaitUntilCommand(lambda: indexer.ready),
                                            commands2.cmd.WaitCommand(5)),
            IntakeOff(intake),
            IndexOff(indexer)
        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)