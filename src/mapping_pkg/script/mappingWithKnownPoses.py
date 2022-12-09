#!/usr/bin/env python3
import rospy
import numpy as np
from mapping_pkg.msg import Readings
from nav_msgs.msg import OccupancyGrid

from utils import euler_from_quaternion , FLTM , RLTM ,Transformation

import math

import tf

'''
Assumptions:
1- Assume msg.pose is the pose of the robot at the time of the sensor reading relative to the real world frame
'''
#TODO: Fix mapping Fuction to correctly map to grid cells , currently it maps to the wrong cells
#TODO: Adjust mapping function to fill un occupied cells with 0

def getGlobalCoords(x,y,transform , robot_pose):
    global t
    #Get the robot pose transformation matrix
    pos = [robot_pose.position.x,robot_pose.position.y,robot_pose.position.z]
    rot = [robot_pose.orientation.x,robot_pose.orientation.y, robot_pose.orientation.z,robot_pose.orientation.w]
    RT = Transformation(parentFrame="" , childFrame="" , pos = pos , rot= rot).transformationMatrix()   
    temp = np.array([x,y,1,1])
    temp = np.matmul(transform,temp)
    temp = np.matmul(t,temp)
    temp = np.matmul(RT,temp)

    x = temp[1]
    y = temp[0]
    return x,y


def inMap(x,y):
    if x >= -50 and x <= 4942 and y >= -50 and y <= 4942:
        return True
    else:
        return False
    # return True


def followRays(Robotpose,ranges , range_max , range_min , start_angle , angle_increment , transform):
    #Check for ray end points to mark cells as occupied
    for i in range(len(ranges)):
        if ranges[i] != "NaN" and ranges[i] <= range_max and ranges[i] >= range_min:
            #Compute the end point of the ray 
            ray_angle = start_angle + (angle_increment*i)

            x = ranges[i]*math.cos(ray_angle)   
            y = ranges[i]*math.sin(ray_angle)
                        
                        
            x,y = getGlobalCoords(x,y,transform,Robotpose)

            #Mark the cell as occupied
            if inMap(x,y):
                cells[int((x+50)/0.02),int((y+50)/0.02)] = 100
    
            #mark the cells that are in the sensor range as free if not occupied
            step_size = ranges[i]/50
            for j in range(50):
                x = step_size*j*math.cos(ray_angle)
                y = step_size*j*math.sin(ray_angle)
                x,y = getGlobalCoords(x,y,transform,Robotpose)
                if inMap(x,y):
                    if cells[int((x+50)/0.02),int((y+50)/0.02)] != 100:
                        cells[int((x+50)/0.02),int((y+50)/0.02)] = 0 


def computeOccupancy(SensorReading):
    
    global cells , resolution

    #Robot Pose
    Robotpose = SensorReading.pose.pose

    #Follow Rays
    followRays(Robotpose,SensorReading.ranges_front,SensorReading.range_max,SensorReading.range_min,SensorReading.start_angle,SensorReading.angle_increment , FLTM)
    followRays(Robotpose,SensorReading.ranges_rear,SensorReading.range_max,SensorReading.range_min,SensorReading.start_angle,SensorReading.angle_increment , RLTM)








def onDataRecived(msg , pub):
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
    computeOccupancy(msg)
    map.data = tuple(cells.flatten())

    pub.publish(map)


def main():

    global cells , resolution
    cells = np.ones((4992,4992),dtype=np.int8)
    cells = cells * -1 # -1 means unknown, 0 means free, 100 means occupied
    resolution = 0.02 #meters
    
    #intializing Node
    rospy.init_node('Mapping')

    #initializing transform Listener
    transform = tf.TransformListener()
    
    #publish The map data to the topic map_data
    pub = rospy.Publisher("map_data",OccupancyGrid,queue_size= 10)

    #subscribe to the aligned Sensor Readings topic
    rospy.Subscriber(name = "/sensor_readings" ,
                    data_class= Readings ,  
                    callback= onDataRecived ,
                    callback_args=(pub),
                    queue_size= 1)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = transform.lookupTransform("robot_odom","robot_base_footprint",rospy.Time(0))
            global t
            t = np.identity(4)
            t = Transformation(parentFrame="robot_odom",childFrame="robot_base_footprint",pos = trans , rot  = rot).transformationMatrix()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass