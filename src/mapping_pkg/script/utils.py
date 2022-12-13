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

