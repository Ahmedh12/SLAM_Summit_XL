#!/usr/bin/env python3
import rospy
import numpy as np
from mapping_pkg.msg import Readings
from nav_msgs.msg import OccupancyGrid

#TODO: Implement the mapping algorithm Given the sensor readings and the odometry data in the Readings message

def onDataRecived(msg , pub):
    cells = np.ones((4992,4992),dtype=np.int8)
    cells = cells * -1 #intially all cells are unknown
    map = OccupancyGrid() #map data to be published

    
    map.header.frame_id = "robot_map"
    map.info.resolution = 0.02
    map.info.width = 4992
    map.info.height = 4992
    map.info.origin.position.x = -50
    map.info.origin.position.y = -50
    map.info.origin.position.z = 0
    map.info.origin.orientation.x = 0
    map.info.origin.orientation.y = 0
    map.info.origin.orientation.z = 0
    map.info.origin.orientation.w = 1

    map.data = tuple(cells.flatten())

    pub.publish(map)

    pass

def main():
    #intializing Node
    rospy.init_node('Mapping')
    
    #publish The map data to the topic map_data
    pub = rospy.Publisher("map_data",OccupancyGrid,queue_size= 10)

    #subscribe to the aligned Sensor Readings topic
    rospy.Subscriber("/sensor_readings" , Readings , onDataRecived , (pub))

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass