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
    is_heading_reversed: bool

    def __init__(self,
                 set_heading_goal: Callable[[float], None],
                 aprilTagNumber: int,
                 apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout,
                 get_xy: Callable[[], Tuple[float, float]],
                 is_heading_reversed: bool):

        super().__init__()
        self._set_heading_goal = set_heading_goal
        self._aprilTagNumber = aprilTagNumber
        self._apriltagfieldlayout = apriltagfieldlayout
        self._get_xy = get_xy
        self.is_heading_reversed = is_heading_reversed


    def execute(self):
        #print("execute turn apriltags")
        tx, ty = self._get_xy()
        pose = self._apriltagfieldlayout.getTagPose(self._aprilTagNumber)
        px = pose.x
        py = pose.y
        heading_goal = math.atan2(ty - py, tx - px)
        if self.is_heading_reversed:
            self._set_heading_goal(heading_goal + math.pi)
        else:
            self._set_heading_goal(heading_goal)
