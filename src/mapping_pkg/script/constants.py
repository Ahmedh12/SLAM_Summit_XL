from nav_msgs.msg import MapMetaData
from utils import Transformation

#Map MetaData
mapMetaData = MapMetaData()
mapMetaData.resolution = 0.02
mapMetaData.width = 4992
mapMetaData.height = 4992
mapMetaData.origin.position.x = -50
mapMetaData.origin.position.y = -50
mapMetaData.origin.position.z = 0
mapMetaData.origin.orientation.x = 0
mapMetaData.origin.orientation.y = 0
mapMetaData.origin.orientation.z = 0
mapMetaData.origin.orientation.w = 1

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