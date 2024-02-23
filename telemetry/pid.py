import ntcore
import wpimath.controller
from config.pid import PIDConfig
from typing import Callable

class PIDEntry:
    """ This is a class that is used to publish PID values to the network table for wpilib PIDControllers.
    This should be built into wpilib, but appears to be broken and we need it right now.  When fixed, it
    should be removed and replaced with the built-in version.  Calling SetData on SmartDashboard and passing
    the software PIDController."""
    _pid: wpimath.controller.ProfiledPIDController | wpimath.controller.PIDController
    _pid_values: PIDConfig
    _izone: float
    _goal: float

    _p_entry: ntcore.DoubleEntry
    _i_entry: ntcore.DoubleEntry
    _d_entry: ntcore.DoubleEntry
    _izone_entry: ntcore.DoubleEntry
    _goal_entry: ntcore.DoubleEntry

    @property
    def izone(self) -> float:
        return self._pid.getIZone()

    @izone.setter
    def izone(self, value: float):
        self._pid.setIZone(value)

    def __init__(self, name: str, pid: wpimath.controller.ProfiledPIDController | wpimath.controller.PIDController):
        self._pid = pid

        self._pid_values = PIDConfig(pid.getP(), pid.getI(), pid.getD())
        self._izone = pid.getIZone()
        self._goal = pid.getGoal().position

        nt_instance = ntcore.NetworkTableInstance.getDefault()
        table_instance = nt_instance.getTable("SmartDashboard").getSubTable(name).getSubTable("PIDController")

        p_topic = table_instance.getDoubleTopic("p")
        i_topic = table_instance.getDoubleTopic("i")
        d_topic = table_instance.getDoubleTopic("d")
        izone_topic = table_instance.getDoubleTopic("izone")
        goal_topic = table_instance.getDoubleTopic("goal")

        p_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        i_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        d_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        izone_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        goal_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))

        self._p_entry = p_topic.getEntry(self._pid_values.p, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._i_entry = i_topic.getEntry(self._pid_values.i, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._d_entry = d_topic.getEntry(self._pid_values.d, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._izone_entry = izone_topic.getEntry(self._izone, ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))
        self._goal_entry = goal_topic.getEntry(self._goal, ntcore.PubSubOptions(keepDuplicates=False,
                                                                                    pollStorage=1))

        self._p_entry.set(self._pid_values.p, 0)
        self._i_entry.set(self._pid_values.i, 0)
        self._d_entry.set(self._pid_values.d, 0)
        self._izone_entry.set(self._izone, 0)
        self._goal_entry.set(self._goal, 0)


    def handle_updates(self, entry: ntcore.DoubleEntry, current_val: float, setter: Callable[[float], None]) -> float:
        result = entry.get()
        if result != current_val:
            #print(f"Setting {result} to {setter}")
            setter(result)
            return result

        return current_val

    def periodic(self):
        # entries support all the same methods as subscribers:
        p = self.handle_updates(self._p_entry, self._pid_values.p, self._pid.setP)
        i = self.handle_updates(self._i_entry, self._pid_values.i, self._pid.setI)
        d = self.handle_updates(self._d_entry, self._pid_values.d, self._pid.setD)
        self._pid_values = PIDConfig(p, i, d)

        self._izone = self.handle_updates(self._izone_entry, self._izone, self._pid.setIZone)

        # setpoint is a special case because it's not a PID value
        setpoint = self._goal_entry.getAtomic()
        if setpoint.time != 0 and self._goal != setpoint.value:
            #print(f"Setting setpoint to {setpoint}")
            self._pid.setGoal(setpoint.value)
            self._goal = setpoint.value

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self._p_entry.unpublish()
        self._i_entry.unpublish()
        self._d_entry.unpublish()
        self._izone_entry.unpublish()
        self._goal_entry.unpublish()

    # often not required in robot code, unless this class doesn't exist for
    # the lifetime of the entire robot program, in which case close() needs to be
    # called to stop subscribing
    def close(self):
        self._p_entry.close()
        self._i_entry.close()
        self._d_entry.close()
        self._izone_entry.close()
        self._goal_entry.close()