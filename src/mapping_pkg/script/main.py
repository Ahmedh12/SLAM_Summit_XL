#!/usr/bin/env python3
import rospy
from sensorModel import SensorModel
from motionModel import MotionModel
from particleFilter import particleFilter
from mapping_pkg.msg import Readings 
from constants import SENSOR_MODEL_PARAMS , MOTION_NOISE_ODOM , MOTION_NOISE_VELOCITY 

def FastSLAM():
    rospy.init_node('FastSLAM', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    n_particles = 2
    motionModel = MotionModel(MOTION_NOISE_ODOM , MOTION_NOISE_VELOCITY)
    sensorModel = SensorModel(SENSOR_MODEL_PARAMS)
    pf = particleFilter(n_particles , motionModel , sensorModel)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/sensor_readings" , Readings)
        if pf.particles == []:
            pf.initParticles(msg)
        else:
            print("update particles")
            pf.updateParticles(msg)
            print("resample particles")
            pf.resampleParticles()
        print("publish map")
        pf.mapPublisher.onDataRecived(pf.map)
        rate.sleep()

def main():
    FastSLAM()
    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass