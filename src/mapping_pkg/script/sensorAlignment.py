#!/usr/bin/env python3
import rospy
from rospy.rostime import Time
import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from mapping_pkg.msg import Readings

'''
/robot/front_laser/scan Type: sensor_msgs/LaserScan 
    link : http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

/robot/rear_laser/scan Type: sensor_msgs/LaserScan  
    link : http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html

/robot/robotnik_base_control/odom Type: nav_msgs/Odometry
    link : http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
'''


def OnDataRecieved(frontLaser,rearLaser,odometry):
    #Sensor Model Readings
    msg = Readings()

    msg.header.frame_id = odometry.header.frame_id

    msg.start_angle = frontLaser.angle_min
    msg.angle_increment = frontLaser.angle_increment
    msg.range_min = frontLaser.range_min
    msg.range_max = frontLaser.range_max

    #all front laser beams plus (310->490) rays from rear laser
    temp_beams_list = list(frontLaser.ranges)
    temp_beams_list.extend(list(rearLaser.ranges)[180:360:1])
    msg.ranges = tuple(temp_beams_list)

    #Motion Model Readings:
    msg.pose = odometry.pose
    end_time = rospy.Time.now()

    global start_time
    global pub
    
    wait = end_time - start_time
    if(wait.secs > 0.5):
        print("Sent: "+str(odometry.pose.pose.position.x))
        start_time = rospy.Time.now()
        pub.publish(msg)
    

def main():
    #intializing Node
    rospy.init_node('sensors_data')
    global start_time
    start_time = rospy.Time.now()

    #subscribing to sensor data topics
    front_laser_sub = message_filters.Subscriber('/robot/front_laser/scan',LaserScan)
    rear_laser_sub = message_filters.Subscriber('/robot/rear_laser/scan',LaserScan)
    odometry_sub = message_filters.Subscriber('/robot/robotnik_base_control/odom',Odometry)
    

    #Aligning Sensor data
    ts = message_filters.ApproximateTimeSynchronizer([front_laser_sub, rear_laser_sub,odometry_sub],10,0.1,allow_headerless=False,reset=True)
    ts.registerCallback(OnDataRecieved)

    #Publishing Sensor data
    global pub
    pub = rospy.Publisher("sensor_readings",Readings,queue_size=1)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass