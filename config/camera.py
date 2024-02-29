import wpimath.geometry as geom
from swerve import swervedrive
from typing import NamedTuple


class CameraConfig(NamedTuple):
    camera_position: geom.Transform3d
    camera_name: str
    swerve_drive: swervedrive
