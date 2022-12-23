#!/usr/bin/env python3
import rospy
from sensorModel import SensorModel
from motionModel import OdometryMotionModel
from particleFilter import particleFilter
from mapping_pkg.msg import Readings 
from constants import SensorModelParams , motionNoise

def FastSLAM():
    rospy.init_node('FastSLAM', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    n_particles = 10
    motionModel = OdometryMotionModel(motionNoise)
    sensorModel = SensorModel(SensorModelParams)
    pf = particleFilter(n_particles , motionModel , sensorModel)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/sensor_readings" , Readings)
        if pf.particles == []:
            pf.initParticles(msg)
            pose = msg.pose.pose
        else:
            print("update particles")
            pf.updateParticles(msg)
            print("resample particles")
            pf.resampleParticles()
            pose = pf.pose
        print("publish map")
        pf.mapPublisher.onDataRecived(pose,msg)
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