import numpy as np
from math import atan2, cos, sin, sqrt, pi
from mappingWithKnownPoses import Mapper
class MeasurementModel:

    def __init__(self , SensorModelParams):
        self.laser_max_range = SensorModelParams['laser_max_range']
        self.laser_min_range = SensorModelParams['laser_min_range']
        self.z_hit = SensorModelParams['z_hit']
        self.z_short = SensorModelParams['z_short']
        self.z_max = SensorModelParams['z_max']
        self.z_rand = SensorModelParams['z_rand']
        self.sigma_hit = SensorModelParams['sigma_hit']
        self.lambda_short = SensorModelParams['lambda_short']
        self.mapper = Mapper()

    
    def _p_hit(self, z, z_t):
        '''
        z: Actual measurement
        z_t: Expected measurement based on ray casting
        '''
        if z < self.laser_min_range or z > self.laser_max_range:
            return 0
        else:
            return (1/(self.sigma_hit * np.sqrt(2 * np.pi))) * \
                np.exp(-0.5 * ((z - z_t)/self.sigma_hit)**2)

    def _p_short(self, z, z_t):
        '''
        z: Actual measurement
        z_t: Expected measurement based on ray casting
        '''
        if z <= z_t and z >= 0:
            return self.lambda_short * np.exp(-self.lambda_short * z)
        else:
            return 0
    
    def _p_max(self, z, z_t):
        if z == z_t:
            return 1
        else:
            return 0

    def _p_rand(self, z, z_t):
        if z < self.laser_max_range and z > self.laser_min_range:
            return 1/z_t
    
    def _p_z(self, z, z_t):
        '''
        z: Actual measurement
        z_t: Expected measurement based on ray casting
        '''
        p_hit = self._p_hit(z, z_t)
        p_short = self._p_short(z, z_t)
        p_max = self._p_max(z, z_t)
        p_rand = self._p_rand(z, z_t)

        probs = np.array([p_hit, p_short, p_max, p_rand])
        factors = np.array([self.z_hit, self.z_short, self.z_max, self.z_rand])

        return factors.T @ probs

    # def p_z_given_x_m(self, z, x_t,m):
    #     '''
    #     z: rays from Sensor
    #     z_t: Expected measurement based on ray casting
    #     x_t: pose of the robot
    #     m: Map
    #     '''
    #     q = 1
    #     for i in range(len(z)):
    #         # compute 


                




