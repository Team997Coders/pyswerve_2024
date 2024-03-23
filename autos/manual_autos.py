import commands2
import commands
from autos import auto_calibrations

def one_note_auto(robot):
    if robot.shooter is None or robot.indexer is None or robot.intake is None or robot.swerve_drive is None:
        raise ValueError("Robot must have a swerve drive, shooter, indexer, and intake to run this auto")

    return commands2.sequentialcommandgroup.SequentialCommandGroup(
        commands2.WaitCommand(20)
    #Taxi
        # commands.DeadReckonX(robot.swerve_drive, -15),
        # commands.DeadReckonY(robot.swerve_drive, 0),
        # commands2.WaitCommand(10)
    #One Note
        # commands.Shoot(robot.shooter, robot.indexer),
        # commands2.WaitCommand(1),
        # commands.DeadReckonX(robot.swerve_drive, -1),
        # commands.DeadReckonY(robot.swerve_drive, 0),
        # commands2.WaitCommand(10)
    #Two Note
        #commands.Shoot(robot.shooter, robot.indexer),
        #commands2.WaitCommand(0.5),
        #commands2.ParallelCommandGroup(
        #   auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, 1),
        #   commands.Load(robot.intake, robot.indexer)
        #),
        #commands2.ParallelCommandGroup(
        #   auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, -1),
        #   commands.Shoot(robot.shooter, robot.indexer)
        #)
    #Three Note
        #commands.Shoot(robot.shooter, robot.indexer),
        #commands2.WaitCommand(0.5),
        #commands2.ParallelCommandGroup(
        #auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, 1),
        #    commands.Load(robot.intake, robot.indexer)
        #),
        #commands2.ParallelCommandGroup(
        #    auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, -1),
        #    commands.Shoot(robot.shooter, robot.indexer)
        #),
        #commands.DeadReckonY(robot.swerve_drive, 1),
        #commands2.ParallelCommandGroup(
        #   commands.DeadReckonX(robot.swerve_drive, 1),
        #   commands.Load(robot.intake, robot.indexer)
        #),
        #commands.DeadReckonX(robot.swerve_drive, -1),
        #commands.DeadReckonY(robot.swerve_drive, -1),
        #commands.Shoot(robot.shooter, robot.indexer),
        #commands2.WaitCommand(0.5),
        #auto_calibrations.create_drive_forward_and_back_auto(robot.swerve_drive, 1)

    )

