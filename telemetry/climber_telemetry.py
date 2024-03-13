from telemetry import FloatEntry
import config
import ntcore


class ClimberTelemetry:
    _config: config.ClimberConfig
    # _intake_entry: FloatEntry
    # _outtake_entry: FloatEntry

    # def set_intake_velocity(self, value: float):
    #     self._config.intake_velocity = value
    #
    # def set_outtake_velocity(self, value: float):
    #     self._config.outtake_velocity = value

    def __init__(self, intake_config: config.ClimberConfig):
        self._config = intake_config

        # self._intake_entry = FloatEntry("Intake", "Intake Velocity",
        #                                            get_value=lambda: self._config.intake_velocity,
        #                                            set_value=self.set_intake_velocity)
        # self._outtake_entry = FloatEntry("Intake", "Outtake Velocity",
        #                                            get_value=lambda: self._config.outtake_velocity,
        #                                            set_value=self.set_outtake_velocity)

    def periodic(self):
        return
        # self._intake_entry.periodic()
        # self._outtake_entry.periodic()