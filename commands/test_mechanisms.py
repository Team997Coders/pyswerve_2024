import commands2
import commands
import subsystems


class TestMechanisms(commands2.Command):
    intake: subsystems.Intake
    shooter: subsystems.Shooter
    indexer: subsystems.Indexer
    # climber: subsystems.Climber
    _command: commands2.Command

    def __init__(self, intake: subsystems.Intake, shooter: subsystems.Shooter, indexer: subsystems.Indexer):
        super().__init__()
        self._intake = intake
        self._shooter = shooter
        self._indexer = indexer
        # self._climber = climber
        self._command = commands2.SequentialCommandGroup(
            commands.IntakeOn(self._intake),
            commands2.WaitCommand(.5),
            commands.IntakeOff(self._intake),
            commands2.WaitCommand(.5),
            commands.SpinupShooter(self._shooter),
            commands2.WaitCommand(.5),
            commands.SpindownShooter(self._shooter),
            commands2.WaitCommand(.5),
            commands.IndexOnIntake(self._indexer),
            commands2.WaitCommand(.5),
            commands.IndexOff(self._indexer),
            commands2.WaitCommand(.5),
            commands.Outtake(self._intake, self._indexer),
            commands2.WaitCommand(.5),
            commands.IndexOff(self._indexer),
            commands.IntakeOff(self._intake),
            commands2.WaitCommand(.5)
        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
