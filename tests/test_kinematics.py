import math
import unittest
from config import ModulePosition
import wpimath.kinematics as kinematics
import wpimath.geometry as geom
import wpimath.units as units

class testKinematics(unittest.TestCase):
  
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.front_left = geom.Translation2d(10, 10)
        self.front_right = geom.Translation2d(10, -10)
        self.back_left = geom.Translation2d(-10, 10)
        self.back_right = geom.Translation2d(-10, -10)
 

    def testModuleOrderOne(self):
        """Test that the module order is correct.  Remember that the positive X direction is away from the team wall"""

        locations = [self.front_left, self.front_right, self.back_left, self.back_right]
        k_fl_fr_bl_br = kinematics.SwerveDrive4Kinematics(*locations)
        chassis_speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(vx=units.meters_per_second(1.0),
                                                                         vy=units.meters_per_second(0.0),
                                                                         omega=units.radians_per_second(0.2),
                                                                         robotAngle=geom.Rotation2d(math.radians(0)))
        module_states = k_fl_fr_bl_br.toSwerveModuleStates(chassis_speed)
        fl, fr, bl, br = module_states

        #We are turning CCW, so the left side wheels should going faster than the right side since the left side has a wider circle
        
        self.printModule(ModulePosition.front_left, fl)
        self.printModule(ModulePosition.front_right, fr)
        self.printModule(ModulePosition.back_left, bl)
        self.printModule(ModulePosition.back_right, br)
        # Wheel should be slower on the left side because they are turning a tighter circle with a smaller radius
        self.assertTrue(fl.speed < fr.speed)
        self.assertTrue(bl.speed < br.speed)
        # Wheel should be deflected more on the left side because they are turning a tighter circle
        self.assertTrue(abs(fl.angle.degrees()) > abs(fr.angle.degrees())) 
        self.assertTrue(abs(bl.angle.degrees()) > abs(br.angle.degrees()))
        # Front wheels and back wheels on a side should be equally deflected, just in different directions
        self.assertAlmostEqual(fl.angle.degrees(), -bl.angle.degrees()) 
        self.assertAlmostEqual(fr.angle.degrees(), -br.angle.degrees()) 
        return 
    
    def testModuleOrderTwo(self):
        """Test that the module order is correct.  This test is the same as TestOne.  The only difference is the order of the modules"""

        locations = [self.front_right, self.front_left, self.back_right, self.back_left]
        k = kinematics.SwerveDrive4Kinematics(*locations)
        chassis_speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(vx=units.meters_per_second(1.0),
                                                                         vy=units.meters_per_second(0.0),
                                                                         omega=units.radians_per_second(0.2),
                                                                         robotAngle=geom.Rotation2d(math.radians(0)))
        module_states = k.toSwerveModuleStates(chassis_speed)
        fr, fl, br, bl = module_states

        #We are turning CCW, so the left side wheels should going faster than the right side since the left side has a wider circle
        
        self.printModule(ModulePosition.front_left, fl)
        self.printModule(ModulePosition.front_right, fr)
        self.printModule(ModulePosition.back_left, bl)
        self.printModule(ModulePosition.back_right, br)
        # Wheel should be slower on the left side because they are turning a tighter circle with a smaller radius
        self.assertTrue(fl.speed < fr.speed)
        self.assertTrue(bl.speed < br.speed)
        # Wheel should be deflected more on the left side because they are turning a tighter circle
        self.assertTrue(abs(fl.angle.degrees()) > abs(fr.angle.degrees())) 
        self.assertTrue(abs(bl.angle.degrees()) > abs(br.angle.degrees()))
        # Front wheels and back wheels on a side should be equally deflected, just in different directions
        self.assertAlmostEqual(fl.angle.degrees(), -bl.angle.degrees()) 
        self.assertAlmostEqual(fr.angle.degrees(), -br.angle.degrees()) 
        return
    
    @staticmethod
    def printModule(pos: ModulePosition, ms: kinematics.SwerveModuleState):
        print(f'{pos} - Speed: {ms.speed} Angle: {ms.angle.degrees()}')
        return
    
    @staticmethod
    def printRotation(r: geom.Rotation2d):
        print(f'Rotation: {r.degrees()}')
        return
    

if __name__ == '__main__':
    unittest.main()