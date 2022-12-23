from math import atan2, cos, sin, sqrt, pi
from utils import normalize_angle
from utils import sample
import numpy as np

class OdometryMotionModel:
    def __init__(self, motionNoise , pre_odom_pose = [0.0, 0.0, 0.0]):
        self.alpha1 = motionNoise["alpha1"]
        self.alpha2 = motionNoise["alpha2"]
        self.alpha3 = motionNoise["alpha3"]
        self.alpha4 = motionNoise["alpha4"]
        self.pre_odom_pose = np.array(pre_odom_pose)
        self.prev_pose = np.array(pre_odom_pose)

    def SampleMotionModel(self, curr_odom_pose):
        '''
        sample motion model based on odometry data and noise parameters alpha1, alpha2, alpha3, alpha4
        The noise is modeled as additive gaussian noise
        :param pre_odom_pose: previous odometry pose
        :param curr_odom_pose: current odometry pose
        :param prev_pose: previous pose
        '''
        x, y, theta = self.pre_odom_pose
        x_new, y_new, theta_new = curr_odom_pose
        x_t_1, y_t_1, theta_t_1 = self.prev_pose

        delta_rot1 = atan2(y_new - y, x_new - x) - theta
        delta_trans = sqrt((x_new - x)**2 + (y_new - y)**2)
        delta_rot2 = theta_new - theta - delta_rot1

        delta_rot1_hat = sample(mean= 0 ,
                                cov= (self.alpha1 * delta_rot1**2 + self.alpha2 * delta_trans**2),
                                type= 'gaussian' ,
                                state= 'univariate' )

        delta_trans_hat = sample(mean= 0 ,
                                cov= (self.alpha3 * delta_trans**2 + self.alpha4 * delta_rot1**2 + self.alpha4 * delta_rot2**2),
                                type= 'gaussian' ,
                                state= 'univariate')
        
        delta_rot2_hat = sample(mean= 0 ,
                                cov= (self.alpha1 * delta_rot2**2 + self.alpha2 * delta_trans**2),
                                type= 'gaussian' ,
                                state= 'univariate')
        
        x_dash = x_t_1 + delta_trans_hat * cos(theta + delta_rot1_hat)
        y_dash = y_t_1 + delta_trans_hat * sin(theta + delta_rot1_hat)
        theta_dash = theta_t_1 + delta_rot1_hat + delta_rot2_hat
        
        theta_dash = normalize_angle(theta_dash)

        x_t = np.array([x_dash, y_dash, theta_dash])

        #print("Curr_pose: ", str(self.prev_pose) ," Sampled_pose: ", str(x_t) , "\npre_odom_pose: ", \
        #    str(self.pre_odom_pose) , " curr_odom_pose: ", str(curr_odom_pose) , "\n")
        

        self.pre_odom_pose = curr_odom_pose
        self.prev_pose = x_t
        return x_t

    

     
