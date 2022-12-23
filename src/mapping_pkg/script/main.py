#!/usr/bin/env python3
import rospy
from mapPublisher import MapPublisher
from constants import mapMetaData , RLTM , FLTM

#to be deleted
#-------------------------------------------------------------------------------
from motionModel import OdometryMotionModel
from sensorModel import SensorModel
from constants import SensorModelParams , motionNoise
from mapping_pkg.msg import Readings
import numpy as np
from scipy.spatial.transform import Rotation as R

def tempCallback(msg):
    global sensorModel , motionModel
    pose = msg.pose.pose
    # pose.position.x = 0
    # pose.position.y = 0
    # pose.position.z = 0
    # pose.orientation.x = 0
    # pose.orientation.y = 0
    # pose.orientation.z = 0
    # pose.orientation.w = 1

    # print(sensorModel.p_z_given_x_m(msg.ranges_front,pose,is_front_laser=True))
    # print(sensorModel.p_z_given_x_m(msg.ranges_rear,pose,is_front_laser=False))

    rot = R.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
    euler = rot.as_euler('xyz',degrees=False)
    motionModel.SampleMotionModel(np.array([pose.position.x,pose.position.y, euler[2]]))
#-------------------------------------------------------------------------------

def main():
    mapper = MapPublisher(publishTopic= "map_data",
            RearLaserTransformMatrix= RLTM,
            FrontLaserTransformMatrix= FLTM,
            mapMetaData= mapMetaData,
            referenceFrame= "robot_map",
            sensorTopic= "/sensor_readings")
    
    #to be deleted
    #---------------------------------------------------------------------------
    global sensorModel
    sensorModel = SensorModel(SensorModelParams= SensorModelParams , mapper = mapper)
    rospy.Subscriber(name = "/sensor_readings" ,
                        data_class= Readings ,  
                        callback= tempCallback ,
                        queue_size= 1)
    
    global motionModel
    motionModel = OdometryMotionModel(motionNoise= motionNoise)
    #---------------------------------------------------------------------------

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass