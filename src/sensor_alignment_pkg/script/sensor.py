#!/usr/bin/env python3
import rospy
from rospy.rostime import Time
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_alignment_pkg.msg import Readings

'''
/robot/front_laser/scan Type: sensor_msgs/LaserScan 
    link : http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

/robot/rear_laser/scan Type: sensor_msgs/LaserScan  
    link : http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

/robot/robotnik_base_control/odom Type: nav_msgs/Odometry
    link : http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
'''

pub = rospy.Publisher("sensor_readings",Readings,queue_size=10)

def OnDataRecieved(frontLaser,rearLaser,odometry):
    #TODO : Fill Readings Message with the appropriate data from Sensor Reading and Odometry 
    #We can Eding msg/Readings.msg if necessary

    msg = Readings()
    pub.publish(msg)
    

def main():
    #intializing Node
    rospy.init_node('sensors_data')
    first = True
    #subscribing to sensor data topics
    front_laser_sub = message_filters.Subscriber('/robot/front_laser/scan',LaserScan)
    rear_laser_sub = message_filters.Subscriber('/robot/rear_laser/scan',LaserScan)
    odometry_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom',Odometry)
    

    #Aligning Sensor data
    ts = message_filters.ApproximateTimeSynchronizer([front_laser_sub, rear_laser_sub,odometry_sub],40,0.05,allow_headerless=False)
    ts.registerCallback(OnDataRecieved)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass