# Packages related to FIRST can be found at https://robotpy.readthedocs.io/en/stable/ as various API's
# If you add a package check if it is listed in the pyproject.toml file to ensure it deploys to the robot
from typing import Optional
import commands2
import commands2.cmd
import subsystems
from commands import index


# Shooting Sequence
# 1a. Spin up motor
# 1b. Turn off indexer
# 2. Wait 200ms
# 3. Turn on indexer
# 4a. Spin down motor
# 4b. Turn off indexer


class SpinupShooter(commands2.InstantCommand):
    _shooter: subsystems.Shooter
    _shot_velocity: float

    def __init__(self, shooter: subsystems.Shooter, shot_velocity: float):
        super().__init__()
        self._shooter = shooter
        self._shot_velocity = shot_velocity

    def execute(self):
        self._shooter.velocity = self._shot_velocity


class SpindownShooter(commands2.InstantCommand):
    _shooter: subsystems.Shooter

    def __init__(self, shooter: subsystems.Shooter):
        super().__init__()
        self._shooter = shooter

    def execute(self):
        self._shooter.velocity = 0


class Shoot(commands2.InstantCommand):
    _command: commands2.Command

    def __init__(self, shooter: subsystems.Shooter, indexer: subsystems.Indexer, shot_velocity: float = 5,
                 spinup_delay: float = 0.2, fire_time: float = 1):
        super().__init__()
        self._command = commands2.cmd.sequence(
            commands2.cmd.ParallelCommandGroup(
                SpinupShooter(shooter, shot_velocity),
                index.IndexOff(indexer)
            ),
            commands2.WaitCommand(spinup_delay),
            index.IndexOn(indexer),
            commands2.WaitCommand(fire_time),
            commands2.cmd.ParallelCommandGroup(
                SpindownShooter(shooter),
                index.IndexOff(indexer)
            )
        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
