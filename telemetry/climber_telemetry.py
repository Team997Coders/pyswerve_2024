import subsystems
from telemetry import FloatEntry
import config
import ntcore
from wpilib import SmartDashboard as sd


class ClimberTelemetry:
    _config: config.ClimberConfig
    _position_entry: FloatEntry
    _climber: subsystems.Climber

    def set_climber_position(self, value: float):
        self._climber.position(value)

    def __init__(self, climber: subsystems.Climber):
        self._climber = climber
        self._config = climber.config

        self._position_entry = FloatEntry("Climber", "Climber Position",
                                          get_value=lambda: self._climber.position,
                                          set_value=self.set_climber_position)

        sd.putBoolean("Climber Sensor", climber.get_climber_sensor_status())

    def periodic(self):
        self._position_entry.periodic()
        sd.putBoolean("Climber Sensor", self._climber.get_climber_sensor_status())