import ntcore
import rev
from config.pid import PIDConfig
from typing import Callable

class SparkMaxPIDEntry:
    """ This is a class that is used to publish PID values to the network table for SparkMaxControllers"""
    _pid: rev.SparkMaxPIDController
    _pid_values: PIDConfig
    _izone: float
    _setpoint: float

    _p_entry: ntcore.DoubleEntry
    _i_entry: ntcore.DoubleEntry
    _d_entry: ntcore.DoubleEntry
    _izone_entry: ntcore.DoubleEntry
    _setpoint_entry: ntcore.DoubleEntry
    _reference_type: rev.CANSparkMax.ControlType

    @property
    def izone(self) -> float:
        return self._pid.getIZone()

    @izone.setter
    def izone(self, value: float):
        self._pid.setIZone(value)


    def __init__(self, name: str, pid: rev.SparkMaxPIDController, reference_type: rev.CANSparkMax.ControlType):
        self._pid = pid
        self._reference_type = reference_type

        self._pid_values = PIDConfig(pid.getP(), pid.getI(), pid.getD())
        self._izone = pid.getIZone()
        self._setpoint = 0

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        table_instance = nt_instance.getTable("SmartDashboard").getSubTable(name).getSubTable("PIDController")

        p_topic = table_instance.getDoubleTopic("p")
        i_topic = table_instance.getDoubleTopic("i")
        d_topic = table_instance.getDoubleTopic("d")
        izone_topic = table_instance.getDoubleTopic("izone")
        setpoint_topic = table_instance.getDoubleTopic("setpoint")

        p_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        i_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        d_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        izone_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        setpoint_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))

        self._p_entry = p_topic.getEntry(self._pid_values.p, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._i_entry = i_topic.getEntry(self._pid_values.i, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._d_entry = d_topic.getEntry(self._pid_values.d, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._izone_entry = izone_topic.getEntry(self._izone, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._setpoint_entry = setpoint_topic.getEntry(self._setpoint, ntcore.PubSubOptions(keepDuplicates=False,
                                                                                            pollStorage=1))

        self._p_entry.set(self._pid_values.p, 0)
        self._i_entry.set(self._pid_values.i, 0)
        self._d_entry.set(self._pid_values.d, 0)
        self._izone_entry.set(self._izone, 0)
        self._setpoint_entry.set(self._setpoint, 0)


    def handle_updates(self, entry: ntcore.DoubleEntry, current_val: float, setter: Callable[[float, int], None]) -> float:
        result = entry.getAtomic()
        if result.time != 0 and result.value != current_val:
            #print(f"Setting {result.value} to {setter}")
            setter(result.value)
            return result.value

        return current_val

    def periodic(self):
        # entries support all the same methods as subscribers:
        p = self.handle_updates(self._p_entry, self._pid_values.p, self._pid.setP)
        i = self.handle_updates(self._i_entry, self._pid_values.i, self._pid.setI)
        d = self.handle_updates(self._d_entry, self._pid_values.d, self._pid.setD)
        self._pid_values = PIDConfig(p, i, d)

        self._izone = self.handle_updates(self._izone_entry, self._izone, self._pid.setIZone)

        # setpoint is a special case because it's not a PID value
        setpoint = self._setpoint_entry.getAtomic()
        if setpoint.time != 0 and self._setpoint != setpoint.value:
            print(f"Setting setpoint to {setpoint}")
            self._pid.setReference(setpoint.value, self._reference_type)
            self._setpoint = setpoint.value

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self._p_entry.unpublish()
        self._i_entry.unpublish()
        self._d_entry.unpublish()
        self._izone_entry.unpublish()
        self._setpoint_entry.unpublish()

    # often not required in robot code, unless this class doesn't exist for
    # the lifetime of the entire robot program, in which case close() needs to be
    # called to stop subscribing
    def close(self):
        self._p_entry.close()
        self._i_entry.close()
        self._d_entry.close()
        self._izone_entry.close()
        self._setpoint_entry.close()