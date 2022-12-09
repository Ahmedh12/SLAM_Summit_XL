import math
from scipy.spatial.transform import Rotation as R
import numpy as np

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



class Transformation:
        def __init__(self, parentFrame ,childFrame ,  pos  , rot):
                self.parentFrame = parentFrame
                self.childFrame = childFrame
                self.pos = pos
                self.rot = rot
        
        def _rotMat(self):
                return R.from_quat(self.rot).as_matrix()

        def _transMat(self):
                return np.array([[1,0,0,self.pos[0]],[0,1,0,self.pos[1]],[0,0,1,self.pos[2]],[0,0,0,1]])
        
        def transformationMatrix(self):
                transMat = self._transMat()
                rotMat = self._rotMat()

                mat = np.concatenate((rotMat,transMat[:-1,3:]),axis=1)
                mat = np.concatenate((mat,[[0,0,0,1]]),axis=0)

                return mat
        def __str__(self) -> str:
                return f"matrix: {self.transformationMatrix()}"


#static Transformations --> The Values and Frame names are taken from the /tf_static topic
# to log the topic run the following command in a terminal:
# $ rostopic echo /tf_static

# Rear Laser
RearLaser = []
RearLaser.append(Transformation(parentFrame="robot_rear_laser_base_link", 
                 childFrame="robot_rear_laser_link", 
                 pos = [0.0, 0.0, 0.116], 
                 rot = [0.0, 0.0, 0.0,1.0]))

RearLaser.append(Transformation(parentFrame="robot_base_link", 
                 childFrame="robot_rear_laser_base_link", 
                 pos = [-0.2865, 0.20894, 0.2973], 
                 rot = [0.3826834323650898, 0.9238795325112867, 0.0,0.0]))
# Front Laser
FrontLaser = []
FrontLaser.append(Transformation(parentFrame="robot_front_laser_base_link", 
                  childFrame="robot_front_laser_link", 
                  pos = [0.0, 0.0, 0.116], 
                  rot = [0.0, 0.0, 0.0,1.0]))

FrontLaser.append(Transformation(parentFrame="robot_base_link", 
                  childFrame="robot_front_laser_base_link", 
                  pos = [0.2865, -0.20894, 0.2973], 
                  rot = [0.9238795325112867, -0.3826834323650899, 0.0,0.0]))

# Base_Link to Base_Footprint
BaseLinkToBaseFrame = Transformation( parentFrame="robot_base_footprint", 
                                      childFrame="robot_base_link", 
                                      pos = [0.0,0.0,0.127], 
                                      rot = [0.0,0.0,0.0,1.0])


RLTM = BaseLinkToBaseFrame.transformationMatrix() @ RearLaser[1].transformationMatrix() @ RearLaser[0].transformationMatrix() 
FLTM = BaseLinkToBaseFrame.transformationMatrix() @ FrontLaser[1].transformationMatrix() @ FrontLaser[0].transformationMatrix()
