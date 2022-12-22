#!/usr/bin/env python3

from mapPublisher import MapPublisher
import rospy
from constants import mapMetaData , RLTM , FLTM

def main():
    MapPublisher(publishTopic= "map_data",
            RearLaserTransformMatrix= RLTM,
            FrontLaserTransformMatrix= FLTM,
            mapMetaData= mapMetaData,
            referenceFrame= "robot_map",
            sensorTopic= "/sensor_readings")

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass