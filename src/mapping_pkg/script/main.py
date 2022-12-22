#!/usr/bin/env python3

from mapPublisher import MapPublisher
import rospy
from constants import mapMetaData , RLTM , FLTM

#to be deleted
from sensorModel import SensorModel
from constants import SensorModelParams
from mapping_pkg.msg import Readings
import numpy as np

def tempCallback(msg):
    global sensorModel
    print(sensorModel.p_z_given_x_m(msg.ranges_front,msg.pose.pose,is_front_laser=True))


def main():
    MapPublisher(publishTopic= "map_data",
            RearLaserTransformMatrix= RLTM,
            FrontLaserTransformMatrix= FLTM,
            mapMetaData= mapMetaData,
            referenceFrame= "robot_map",
            sensorTopic= "/sensor_readings")
    
    #to be deleted
    global sensorModel
    sensorModel = SensorModel(SensorModelParams= SensorModelParams)
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