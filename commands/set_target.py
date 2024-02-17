import math
from typing import Callable
import commands2


class SetTarget(commands2.Command):
    _set_heading_goal: Callable[[float], None]
    _get_target_xy: Callable[[], tuple[float, float]]
    _get_chassis_xy: Callable[[], tuple[float, float]]

    def __init__(self, set_heading_goal: Callable[[float], None],
                 get_chassis_xy: Callable[[], tuple[float, float]],
                 get_target_xy: Callable[[], tuple[float, float]]):
        super().__init__()
        self._set_heading_goal = set_heading_goal
        self._get_chassis_xy = get_chassis_xy
        self._get_target_xy = get_target_xy

    def execute(self):
        tx, ty = self._get_target_xy()
        px, py = self._get_chassis_xy()
        heading_goal = math.atan2(ty - py, tx - px)
        # TODO: the math to get the heading and set it on the HeadingTracker
        self._set_heading_goal(heading_goal)
