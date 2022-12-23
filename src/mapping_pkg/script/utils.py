from scipy.spatial.transform import Rotation as R
import numpy as np

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


def sample(mean, cov,type = 'gaussian', state = 'univariate'):
        if type == 'gaussian':
                if state == 'multivariate':
                        return np.random.multivariate_normal(mean, cov)
                elif state == 'univariate':
                        return np.random.normal(mean, cov)
        elif type == 'triangular':
                return np.random.triangular(mean, cov)

def normalize_angle(angle):
        while angle > np.pi:
                angle -= 2*np.pi
        while angle < -np.pi:
                angle += 2*np.pi
        return angle


def z_quat_to_euler(pose):
        rot = R.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        euler = rot.as_euler('xyz',degrees=False)
        return euler[2]

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]