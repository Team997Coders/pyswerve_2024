import commands2
import commands
from typing import Sequence
import subsystems
def shoot_drive_load_backup_auto(robot) -> Sequence[commands2.Command]:

    if robot.shooter is None or robot.indexer is None or robot.intake is None:
        raise ValueError("Robot must have a shooter, indexer, and intake to run this auto")

    # cmd = commands2.cmd.ParallelRaceGroup(
    #         commands2.cmd.sequence(
    #             commands.Shoot(robot.shooter, robot.indexer),
    #             commands2.cmd.WaitCommand(
    #                 robot.config.shooter_config.default_spinup_delay + robot.config.shooter_config.default_fire_time),
    #             commands2.cmd.ParallelCommandGroup(
    #                 commands.Load(robot.intake, robot.indexer),
    #                 #.cmd.sequence(
    #                     # commands.GotoXYTheta(robot.swerve_drive, (2, 0, 0),
    #                     #                      robot._x_axis_control, robot._y_axis_control, robot._heading_control),
    #                     # commands.GotoXYTheta(robot.swerve_drive, (-2, 0, 0),
    #                     #                      robot._x_axis_control, robot._y_axis_control, robot._heading_control),
    #                 #),
    #             ),
    #         ),
    #         commands2.cmd.WaitCommand(15)
    #     )
    cmd = commands2.cmd.sequence(
            commands2.cmd.sequence(
                commands.Shoot(robot.shooter, robot.indexer),
                commands2.cmd.WaitCommand(1)
            ),
            commands2.cmd.sequence(
                commands.GotoXYTheta(robot.swerve_drive, (3, 0, 0)),
                commands2.cmd.ParallelCommandGroup(
                    commands2.cmd.sequence(
                        commands.Load(robot.intake, robot.indexer),
                        commands.GotoXYTheta(robot.swerve_drive, (-3, 0, 0))
                    )
                )
            ),
            commands2.cmd.sequence(
                commands.Shoot(robot.shooter, robot.indexer),
                commands2.cmd.WaitCommand(1)
            ),
            commands2.cmd.sequence(
                commands.GotoXYTheta(robot.swerve_drive, (3, 2, 0)),
                commands2.cmd.ParallelCommandGroup(
                    commands2.cmd.sequence(
                        commands.Load(robot.intake, robot.indexer),
                        commands.GotoXYTheta(robot.swerve_drive, (-3, -2, 0))
                    )
                ),
                commands.Shoot(robot.shooter, robot.indexer)
            )
    )
    return cmd.execute()
