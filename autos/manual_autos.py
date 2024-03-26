import commands2
import wpilib
import commands
from autos import auto_calibrations



def one_note_auto(robot):
    if robot.shooter is None or robot.indexer is None or robot.intake is None or robot.swerve_drive is None:
        raise ValueError("Robot must have a swerve drive, shooter, indexer, and intake to run this auto")

    
