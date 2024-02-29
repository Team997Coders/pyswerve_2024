import commands
import commands2
import swerve
import subsystems


def create_drive_forward_and_back_auto(swerve_drive: swerve.SwerveDrive,
                                       x_axis_control: subsystems.AxisPositionControl,
                                       y_axis_control: subsystems.AxisPositionControl,
                                       heading_control: subsystems.ChassisHeadingControl):

    starting_pos = swerve_drive.pose.x, swerve_drive.pose.y
    target = (starting_pos[0] + 3, starting_pos[1])

    print(f'Going between {starting_pos} and {target}')
    cmd = commands2.cmd.repeatingSequence(
        commands.GotoXYTheta(swerve_drive, (target[0], target[1], 0),
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, (starting_pos[0], starting_pos[1], 0),
                             x_axis_control, y_axis_control, heading_control))

    drv_cmd = commands.drive.Drive(
        swerve_drive,
        get_x=lambda: x_axis_control.desired_velocity,
        get_y=lambda: y_axis_control.desired_velocity,
        get_theta=lambda: heading_control.desired_velocity
    )
    cmd.requirements = {x_axis_control, y_axis_control, heading_control}
    drv_cmd.requirements = {swerve_drive}

    return cmd, drv_cmd

