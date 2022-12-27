from nav_msgs.msg import MapMetaData
from utils import Transformation

#Reference Frame
REFERENCE_FRAME = "robot_map"

#Map MetaData
MAP_META_DATA = MapMetaData()
MAP_META_DATA.resolution = 0.02
MAP_META_DATA.width = 4992
MAP_META_DATA.height = 4992
MAP_META_DATA.origin.position.x = -50
MAP_META_DATA.origin.position.y = -50
MAP_META_DATA.origin.position.z = 0
MAP_META_DATA.origin.orientation.x = 0
MAP_META_DATA.origin.orientation.y = 0
MAP_META_DATA.origin.orientation.z = 0
MAP_META_DATA.origin.orientation.w = 1

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


#Transformation Matrices for the static transformations of laser sensors
RLTM = BaseLinkToBaseFrame.transformationMatrix() @ RearLaser[1].transformationMatrix() @ RearLaser[0].transformationMatrix() 
FLTM = BaseLinkToBaseFrame.transformationMatrix() @ FrontLaser[1].transformationMatrix() @ FrontLaser[0].transformationMatrix()

#motion Noise parameters
MOTION_NOISE_ODOM = {}
MOTION_NOISE_ODOM['alpha1'] =  0
MOTION_NOISE_ODOM['alpha2'] =  0
MOTION_NOISE_ODOM['alpha3'] =  0
MOTION_NOISE_ODOM['alpha4'] =  0

# MOTION_NOISE_ODOM = {}
# MOTION_NOISE_ODOM['alpha1'] =  1e-3
# MOTION_NOISE_ODOM['alpha2'] =  1e-3
# MOTION_NOISE_ODOM['alpha3'] =  1e-3
# MOTION_NOISE_ODOM['alpha4'] =  1e-3

MOTION_NOISE_VELOCITY = {}
MOTION_NOISE_VELOCITY['alpha1'] = 1e-3
MOTION_NOISE_VELOCITY['alpha2'] = 1e-3
MOTION_NOISE_VELOCITY['alpha3'] = 1e-4
MOTION_NOISE_VELOCITY['alpha4'] = 1e-4
MOTION_NOISE_VELOCITY['alpha5'] = 0.1
MOTION_NOISE_VELOCITY['alpha6'] = 0.1

#Sensor Model parameters
SENSOR_MODEL_PARAMS = {}
SENSOR_MODEL_PARAMS['z_hit'] = 0.99
SENSOR_MODEL_PARAMS['z_short'] = 5e-3
SENSOR_MODEL_PARAMS['z_max'] = 25e-4
SENSOR_MODEL_PARAMS['z_rand'] = 25e-4
SENSOR_MODEL_PARAMS['sigma_hit'] = 0.1
SENSOR_MODEL_PARAMS['lambda_short'] = 0.1
SENSOR_MODEL_PARAMS['laser_max_range'] = 30
SENSOR_MODEL_PARAMS['laser_min_range'] = 0.1
SENSOR_MODEL_PARAMS['start_angle'] = -2.3561999797821045
SENSOR_MODEL_PARAMS['angle_increment'] = 0.008726666681468487
