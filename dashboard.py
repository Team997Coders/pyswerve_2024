from wpilib import SmartDashboard


def getNumberFromDashboard(key: str, defaultValue: float):
       returnedValue = SmartDashboard.getNumber(key, defaultValue)
       return returnedValue
    
def assignValue(assignedKey: str, assignedValue: float):
        SmartDashboard.putNumber(assignedKey, assignedValue)
    

        