import subsystems
from telemetry import FloatEntry
import config
from wpilib import SmartDashboard


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

    def periodic(self):
        self._position_entry.periodic()
        SmartDashboard.putBoolean("Climber Sensor", self._climber.get_climber_sensor_status())
