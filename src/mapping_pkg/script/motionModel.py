from math import atan2, cos, sin, sqrt
from utils import normalize_angle , z_quat_to_euler
from utils import sample , get_quaternion_from_euler
from geometry_msgs.msg import Pose
import numpy as np

class MotionModel:
    def __init__(self, motionNoiseOdom , motionNoiseVelocity):
        self.motionNoiseOdom = motionNoiseOdom
        self.motionNoiseVelocity = motionNoiseVelocity
        self.last_time = 0.0


    def sampleMotionModelOdom(self, prev_odom_pose ,curr_odom_pose , prev_pose):
        '''
        sample motion model based on odometry data and noise parameters alpha1, alpha2, alpha3, alpha4
        The noise is modeled as additive gaussian noise
        :param pre_odom_pose: previous odometry pose
        :param curr_odom_pose: current odometry pose
        :param prev_pose: previous pose
        '''

        alpha1 = self.motionNoiseOdom["alpha1"]
        alpha2 = self.motionNoiseOdom["alpha2"]
        alpha3 = self.motionNoiseOdom["alpha3"]
        alpha4 = self.motionNoiseOdom["alpha4"]

        x_new, y_new, theta_new = curr_odom_pose.position.x, curr_odom_pose.position.y,z_quat_to_euler(curr_odom_pose)
        x, y, theta = prev_odom_pose.position.x, prev_odom_pose.position.y,z_quat_to_euler(prev_odom_pose)
        x_t_1, y_t_1, theta_t_1 = prev_pose.position.x, prev_pose.position.y,z_quat_to_euler(prev_pose)

        delta_rot1 = atan2(y_new - y, x_new - x) - theta
        delta_trans = sqrt((x_new - x)**2 + (y_new - y)**2)
        delta_rot2 = theta_new - theta - delta_rot1

        delta_rot1_hat = delta_rot1 - sample(mean= 0 ,
                                            cov= (alpha1 * delta_rot1**2 + alpha2 * delta_trans**2),
                                            type= 'gaussian' ,
                                            state= 'univariate' )

        delta_trans_hat = delta_trans - sample(mean= 0 ,
                                              cov= (alpha3 * delta_trans**2 + alpha4 * delta_rot1**2 + alpha4 * delta_rot2**2),
                                              type= 'gaussian' ,
                                              state= 'univariate')
        
        delta_rot2_hat = delta_rot2 - sample(mean= 0 ,
                                            cov= (alpha1 * delta_rot2**2 + alpha2 * delta_trans**2),
                                            type= 'gaussian' ,
                                            state= 'univariate')
        
        # theta_norm = normalize_angle(theta + delta_rot1_hat)
        theta_norm = theta + delta_rot1_hat

        x_dash = x_t_1 + delta_trans_hat * cos(theta_norm)
        y_dash = y_t_1 + delta_trans_hat * sin(theta_norm)
        theta_dash = theta_t_1 + delta_rot1_hat + delta_rot2_hat
        
        # theta_dash = normalize_angle(theta_dash)
        x_t = Pose()
        x_t.position.x = x_dash 
        x_t.position.y = y_dash 
        x_t.position.z = 0
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,theta_dash)
        x_t.orientation.x = qx
        x_t.orientation.y = qy
        x_t.orientation.z = qz
        x_t.orientation.w = qw

        return x_t

    
    def sampleMotionModelVelocity(self, control , t , prev_pose):
        '''
        sample motion model based on velocity data and noise parameters alpha1, alpha2, alpha3, alpha4 , alpha5, alpha6
        The noise is modeled as additive gaussian noise
        :param control: velocity data
        :param prev_pose: previous pose
        '''

        alpha1 = self.motionNoiseVelocity["alpha1"]
        alpha2 = self.motionNoiseVelocity["alpha2"]
        alpha3 = self.motionNoiseVelocity["alpha3"]
        alpha4 = self.motionNoiseVelocity["alpha4"]
        alpha5 = self.motionNoiseVelocity["alpha5"]
        alpha6 = self.motionNoiseVelocity["alpha6"]


        x_t_1, y_t_1, theta_t_1 = prev_pose.position.x, prev_pose.position.y,z_quat_to_euler(prev_pose)
        v, w = control[0], control[1]
        v_hat = v + sample(mean= 0 ,
                           cov= (alpha1 * v**2 + alpha2 * w**2),
                           type= 'gaussian' ,
                           state= 'univariate')
        w_hat = w + sample(mean= 0 ,
                           cov= (alpha3 * v**2 + alpha4 * w**2),
                           type= 'gaussian' ,
                           state= 'univariate')
        gamma_hat = sample(mean= 0 ,
                           cov= (alpha5 * v**2 + alpha6 * w**2),
                           type= 'gaussian' ,
                           state= 'univariate')

        ratio = v_hat / w_hat
        delta_t = t - self.last_time
        x_dash = x_t_1 - ratio * sin(theta_t_1) + ratio * sin(theta_t_1 + (w_hat * delta_t))
        y_dash = y_t_1 + ratio * cos(theta_t_1) - ratio * cos(theta_t_1 + (w_hat * delta_t))
        theta_dash = theta_t_1 + (w_hat * delta_t) + gamma_hat * delta_t

        x_t = Pose()
        x_t.position.x = x_dash
        x_t.position.y = y_dash
        x_t.position.z = 0
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,theta_dash)
        x_t.orientation.x = qx
        x_t.orientation.y = qy
        x_t.orientation.z = qz
        x_t.orientation.w = qw

        self.last_time = t

        return x_t

    

     
