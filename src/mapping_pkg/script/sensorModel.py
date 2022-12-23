import numpy as np
from math import atan2, cos, sin, sqrt, pi
from mapper import Mapper
from constants import RLTM , FLTM , mapMetaData

class SensorModel:

    def __init__(self , SensorModelParams , mapper = None):
        self.laser_max_range = SensorModelParams['laser_max_range']
        self.laser_min_range = SensorModelParams['laser_min_range']
        self.z_hit = SensorModelParams['z_hit']
        self.z_short = SensorModelParams['z_short']
        self.z_max = SensorModelParams['z_max']
        self.z_rand = SensorModelParams['z_rand']
        self.sigma_hit = SensorModelParams['sigma_hit']
        self.lambda_short = SensorModelParams['lambda_short']
        self.start_angle = SensorModelParams['start_angle']
        self.angle_increment = SensorModelParams['angle_increment']
        if mapper is None:
            self.mapper = Mapper(RearLaserTransformMatrix= RLTM,
                                 FrontLaserTransformMatrix= FLTM,
                                 mapMetaData= mapMetaData)
        else:
            self.mapper = mapper
    
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
        '''
        z: Actual measurement
        z_t: Expected measurement based on ray casting
        '''
        if z == z_t:
            return 1
        else:
            return 0

    def _p_rand(self, z, z_t):
        '''
        z: Actual measurement
        z_t: Expected measurement based on ray casting
        '''
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

    def _expected_measurement(self, x_t, i , actual , is_front_laser = True ):
        '''
        x_t: pose of the robot
        i: ray index
        '''
        # computes the expected measurement wrt the robot pose 
        # in the odom frame according to the map constructed so far
        ray_angle = self. start_angle + i * self.angle_increment
        if is_front_laser:
            z_t = self.mapper.ray_casting(x_t, ray_angle , actual , FLTM)
        else:
            z_t = self.mapper.ray_casting(x_t, ray_angle , self.laser_max_range , RLTM)
        return z_t

    def p_z_given_x_m(self, z, x_t , mapper ,is_front_laser = True):
        '''
        z: rays from Sensor
        z_t: Expected measurement based on robot pose and map
        x_t: pose of the robot in the odom frame
        mapper: map constructed so far
        '''
        self.mapper = mapper
        q = 0
        for i in range(len(z)):
            # compute the expected measurement
            if z[i] != "inf" and z[i] > self.laser_min_range and z[i] < self.laser_max_range:
                z_t = self._expected_measurement(x_t, i , z[i] , is_front_laser)
                # print("expected: " , z_t , "  actual: " , z[i])
                q += self._p_z(z[i], z_t)
        return q


                




