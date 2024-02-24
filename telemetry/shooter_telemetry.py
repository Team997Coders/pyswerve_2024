from telemetry import FloatEntry
import config
import ntcore

class ShooterTelemetry:

    _config: config.ShooterConfig

    def set_default_fire_time(self, value: float):
        self._config.default_fire_time = value

    def set_default_velocity(self, value: float):
        self._config.default_velocity = value

    def set_spinup_delay(self, value: float):
        self._config.default_spinup_delay = value

    def __init__(self, shooter_config: config.ShooterConfig):
        self._config = shooter_config

        self._default_fire_time_entry = FloatEntry("Shooter", "Default Fire Time",
                                                   get_value=lambda: self._config.default_fire_time,
                                                   set_value=self.set_default_fire_time)
        self._default_velocity_entry = FloatEntry("Shooter", "Default Velocity",
                                                   get_value=lambda: self._config.default_velocity,
                                                   set_value=self.set_default_velocity)
        self._default_spinup_entry = FloatEntry("Shooter", "Spinup Delay",
                                                   get_value=lambda: self._config.default_spinup_delay,
                                                   set_value=self.set_spinup_delay)

    def periodic(self):
        self._default_velocity_entry.periodic()
        self._default_fire_time_entry.periodic()
        self._default_spinup_entry.periodic()