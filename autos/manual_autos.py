import commands2
import commands
from typing import Sequence
from autos import auto_calibrations


def two_note_auto(robot):
    if robot.shooter is None or robot.indexer is None or robot.intake is None:
        raise ValueError("Robot must have a shooter, indexer, and intake to run this auto")
    return commands2.cmd.sequence(
        commands2.cmd.sequence(
            commands.Shoot(robot.shooter, robot.indexer),
            commands2.WaitCommand(0.5),
            commands2.ParallelCommandGroup(
                auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, 1),
                commands.Load(robot.intake, robot.indexer)
            ),
            commands2.ParallelCommandGroup(
                auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, -1),
                commands.Shoot(robot.shooter, robot.indexer)
            )
        )
    )



def taxi(robot):
    if robot.shooter is None or robot.indexer is None or robot.intake is None or robot.swerve_drive is None:
        raise ValueError("Robot must have a swerve drive, shooter, indexer, and intake to run this auto")
    return commands2.cmd.sequence(
        commands.Shoot(robot.shooter, robot.indexer),
        commands2.WaitCommand(0.5),
        auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, 1),
    )

