#!/usr/bin/env python3

from mapPublisher import MapPublisher
import rospy
from constants import mapMetaData , RLTM , FLTM

#to be deleted
from sensorModel import SensorModel
from constants import SensorModelParams
from mapping_pkg.msg import Readings


def tempCallback(msg):
    global sensorModel
    pose = msg.pose.pose
    # pose.position.x = 0
    # pose.position.y = 0
    # pose.position.z = 0
    # pose.orientation.x = 0
    # pose.orientation.y = 0
    # pose.orientation.z = 0
    # pose.orientation.w = 1

    print(sensorModel.p_z_given_x_m(msg.ranges_front,pose,is_front_laser=True))


def main():
    mapper = MapPublisher(publishTopic= "map_data",
            RearLaserTransformMatrix= RLTM,
            FrontLaserTransformMatrix= FLTM,
            mapMetaData= mapMetaData,
            referenceFrame= "robot_map",
            sensorTopic= "/sensor_readings")
    
    #to be deleted
    global sensorModel
    sensorModel = SensorModel(SensorModelParams= SensorModelParams , mapper = mapper)
    rospy.Subscriber(name = "/sensor_readings" ,
                        data_class= Readings ,  
                        callback= tempCallback ,
                        queue_size= 1)


    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass