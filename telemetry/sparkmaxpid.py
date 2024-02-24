import ntcore
import rev

class SparkMaxPIDEntry(ntcore.NTSendable):
    """ This is a class that is used to publish PID values to the network table for SparkMaxControllers"""
    _pid: rev.SparkMaxPIDController
    _setpoint: float

    def initSendable(self, builder: ntcore.NTSendableBuilder) -> None:
        builder.setSmartDashboardType("PIDController")
        builder.addDoubleProperty("p", lambda: self._pid.getP(), lambda x: self._pid.setP(x))
        builder.addDoubleProperty("i", lambda: self._pid.getI(), lambda x: self._pid.setI(x))
        builder.addDoubleProperty("d", lambda: self._pid.getD(), lambda x: self._pid.setD(x))
        builder.addDoubleProperty("izone", lambda: self._pid.getIZone(), lambda x: self._pid.setIZone(x))
        builder.addDoubleProperty("setpoint", getter=lambda: self.setpoint, setter=lambda x: self.set_setpoint(x))

    @property
    def setpoint(self) -> float:
        return self._setpoint

    def set_setpoint(self, value: float):
        self._pid.setReference(value, self._reference_type)
        self._setpoint = value

    def __init__(self, name: str, pid: rev.SparkMaxPIDController, reference_type: rev.CANSparkMax.ControlType):
        super().__init__()
        self._pid = pid
        self._reference_type = reference_type
        self._setpoint = 0
