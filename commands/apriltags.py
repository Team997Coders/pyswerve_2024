import commands2
import robotpy_apriltag
import commands
from typing import Callable, Tuple
import math

import subsystems.chassis_heading_control


class AprilTagPointer(commands2.Command):

    _set_heading_goal: Callable[[float], None]
    _aprilTagNumber: int
    _apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    _get_xy: Callable[[], Tuple[float, float]]

    def __init__(self,
                 set_heading_goal: Callable[[float], None],
                 aprilTagNumber: int,
                 apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout,
                 get_xy: Callable[[], Tuple[float, float]]):

        super().__init__()
        self._set_heading_goal = set_heading_goal
        self._aprilTagNumber = aprilTagNumber
        self._apriltagfieldlayout = apriltagfieldlayout
        self._get_xy = get_xy


    def execute(self):
        #print("execute turn apriltags")
        tx, ty = self._get_xy()
        pose = self._apriltagfieldlayout.getTagPose(self._aprilTagNumber)
        px = pose.x
        py = pose.y
        heading_goal = math.atan2(ty - py, tx - px)
        # TODO: the math to get the heading and set it on the HeadingTracker
        self._set_heading_goal(heading_goal)


