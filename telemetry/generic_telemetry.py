
from typing import Callable
import ntcore


class FloatEntry:
    """ This is a class that is used to publish float values to the network table"""
    _value: float
    _get_value: Callable[[], float]
    _set_value: Callable[[float], None]

    _value_entry: ntcore.DoubleEntry

    def __init__(self, subtable: str, name: str, get_value: Callable[[], float] | None, set_value: Callable[[float], None], default_value: float | None = 0.0):
        self._value = get_value() if get_value is not None else default_value
        self._get_value = get_value
        self._set_value = set_value
        nt_instance = ntcore.NetworkTableInstance.getDefault()
        table_instance = nt_instance.getTable("SmartDashboard").getSubTable(subtable)

        value_topic = table_instance.getDoubleTopic(name)

        value_topic.publish(ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))

        self._value_entry = value_topic.getEntry(self._value,
                                             ntcore.PubSubOptions(keepDuplicates=False, pollStorage=1))

        self._value_entry.set(self._value, 0)

    def handle_updates(self, entry: ntcore.DoubleEntry, current_val: float,
                       setter: Callable[[float], None]) -> float:
        result = entry.get()
        if result != current_val:
            # print(f"Setting {result} to {setter}")
            setter(result)
            return result

        return current_val

    def periodic(self):
        # entries support all the same methods as subscribers:
        self._value = self.handle_updates(self._value_entry, self._value, self._set_value)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self._value_entry.unpublish()

    # often not required in robot code, unless this class doesn't exist for
    # the lifetime of the entire robot program, in which case close() needs to be
    # called to stop subscribing
    def close(self):
        self._value_entry.close()
