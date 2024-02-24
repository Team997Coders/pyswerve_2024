from telemetry import FloatEntry
import config
import ntcore


class IndexerTelemetry:
    _config: config.IndexerConfig
    _velocity_entry: FloatEntry
    _shoot_entry: FloatEntry
    _outtake_entry: FloatEntry

    def set_intake_velocity(self, value: float):
        self._config.intake_velocity = value

    def set_shoot_velocity(self, value: float):
        self._config.shoot_velocity = value

    def set_outtake_velocity(self, value: float):
        self._config.outtake_velocity = value

    def __init__(self, indexer_config: config.IndexerConfig):
        self._config = indexer_config

        self._velocity_entry = FloatEntry("Indexer", "Intake Velocity",
                                                   get_value=lambda: self._config.intake_velocity,
                                                   set_value=self.set_intake_velocity)
        self._shoot_entry = FloatEntry("Indexer", "Shoot Velocity",
                                                   get_value=lambda: self._config.shoot_velocity,
                                                   set_value=self.set_shoot_velocity)
        self._outtake_entry = FloatEntry("Indexer", "Outtake Velocity",
                                                   get_value=lambda: self._config.outtake_velocity,
                                                   set_value=self.set_outtake_velocity)

    def periodic(self):
        self._velocity_entry.periodic()
        self._shoot_entry.periodic()
        self._outtake_entry.periodic()