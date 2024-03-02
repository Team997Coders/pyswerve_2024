import wpimath.geometry as geom
from typing import NamedTuple


class PhotonCameraConfig(NamedTuple):
    camera_position: geom.Transform3d
    camera_name: str


class LimelightCameraConfig(NamedTuple):
    camera_position: geom.Transform3d
    refresh_rate: float  # How often we attempt to update the pose in Hz
    camera_name: str | None = None # Use the default 'limelight' name if this is None
