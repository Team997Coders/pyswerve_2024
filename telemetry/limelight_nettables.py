import ntcore
from ntcore import NetworkTableInstance
from typing import Tuple, Any, Sequence
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d
import wpilib


class LimeLightNetTables:
    """Inspired by FROG 3160's Limelight class, this class reads pose data directly from net-tables"""

    _botpose = Pose3d | None
    network_table: ntcore.NetworkTable

    _last_botpose_update: float | None = None
    _last_targetpose_update: float | None = None

    def __init__(self, limelight_name: str | None = None):
        limelight_name = "limelight" if limelight_name is None else limelight_name
        self._last_botpose_update = None
        self._last_targetpose_update = None

        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=limelight_name
        )

        self.nt_botpose = self.network_table.getFloatArrayTopic("botpose").subscribe(
            [-997, -997, -997, 0, 0, 0, -1]
        )

        #self.network_table.addListener("botpose",  self._on_botpose_change)

        self.nt_botpose_blue = self.network_table.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-997, -997, -997, 0, 0, 0, -1])
        self.nt_botpose_red = self.network_table.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-997, -997, -997, 0, 0, 0, -1])
        self.nt_targetpose_robotspace = self.network_table.getFloatArrayTopic(
            "targetpose_robotspace"
        ).subscribe([-997, -997, -997, 0, 0, 0, -1])
        self.nt_pipeline = self.network_table.getIntegerTopic("getpipe").subscribe(-1)
        # create the timer that we can use to timestamp
        self.timer = wpilib.Timer()

    # def _on_botpose_change(self, table: ntcore.NetworkTable, key: str, event: ntcore.Event):
    #     if isinstance(event.data, ntcore.ValueEventData):
    #         event.data.value.getFloatArray()
    #         event.data.value.time()
    #     self._botpose = self.array_to_bot_pose_estimate(value)

    @property
    def PipelineNum(self) -> int:
        return self.nt_pipeline.get()

    @PipelineNum.setter
    def PipelineNum(self, pipeline_num: int):
        self.network_table.putNumber("pipeline", pipeline_num)

    def getBotPoseEstimate(self) -> Tuple[Pose3d | None, float | None]:
        value = self.nt_botpose.getAtomic()
        if value.time == self._last_botpose_update:
            return None, None

        self._last_botpose_update = value.time
        return self.array_to_bot_pose_estimate(value)

    def getBotPoseEstimateBlue(self) -> Tuple[Pose3d | None, float | None]:
        return self.array_to_bot_pose_estimate(self.nt_botpose_blue.getAtomic())

    def getBotPoseEstimateRed(self) -> Tuple[Pose3d | None, float | None]:
        return self.array_to_bot_pose_estimate(self.nt_botpose_red.getAtomic())

    def getTargetTransform(self) -> Tuple[Transform3d, float] | Tuple[None, None]:
        transform_array = self.nt_targetpose_robotspace.getAtomic()
        if transform_array.time == 0 or transform_array.time == self._Last_targetpose_update:
            return None, None

        self._Last_targetpose_update = transform_array.time

        timestamp_us = transform_array.time
        latency_ms = transform_array.value[6]
        transform = Transform3d(
            Translation3d(transform_array[0], transform_array[1], transform_array[2]),
            Rotation3d(transform_array[3], transform_array[4], transform_array[5]),
        )
        if timestamp_us > 0:
            return transform, timestamp_us - (latency_ms / 1000)

        return None, None

    def array_to_bot_pose_estimate(self, pose_array_timestamped: ntcore.TimestampedFloatArray) -> Tuple[
        Pose3d | None, float | None]:
        """Takes limelight array data and creates a Pose3d object for
           robot position and a timestamp reprepresenting the time
           the position was observed.

        Args:
            poseArray (_type_): An array from the limelight network tables.

        Returns:
            Tuple[Pose3d, Any]: Returns vision Pose3d and timestamp.
        """
        pose_array = pose_array_timestamped.value
        pose_update_us = pose_array_timestamped.time
        if pose_update_us == 0:
            return None, None

        pX, pY, pZ, pRoll, pPitch, pYaw, latency_ms = pose_array
        if latency_ms == -1:
            return None, None
        else:
            timestamp_us =  pose_update_us - (latency_ms / 1000)
            timestamp_sec = timestamp_us / 1000000
            return Pose3d(
                Translation3d(pX, pY, pZ), Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
            ), timestamp_sec
