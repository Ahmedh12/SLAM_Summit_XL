#!/usr/bin/env python3
import rospy
import numpy as np
from mapping_pkg.msg import Readings
from nav_msgs.msg import OccupancyGrid
import math
 
#Global Variables
cells = np.ones((4992,4992),dtype=np.int8)
cells = cells * -1 # -1 means unknown, 0 means free, 100 means occupied
resolution = 0.02 #meters


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



'''
Assumptions:
1- Assume msg.pose is the pose of the robot at the time of the sensor reading relative to the real world frame
'''
#TODO: Fix mapping Fuction to correctly map to grid cells , currently it maps to the wrong cells
#TODO: Adjust mapping function to fill un occupied cells with 0



def computeOccupancy(SensorReading):
    global si , fr
    
    #Robot Pose
    x = SensorReading.pose.pose.position.x 
    y = SensorReading.pose.pose.position.y 
    theta = euler_from_quaternion(SensorReading.pose.pose.orientation.x,SensorReading.pose.pose.orientation.y,SensorReading.pose.pose.orientation.z,SensorReading.pose.pose.orientation.w)[2]
    Robotpose = [x,y,theta]

    print("Recived: "+str(SensorReading.pose.pose.position.x))

    #Check for ray end points to mark cells as occupied
    for i in range(len(SensorReading.ranges)):
        if SensorReading.ranges[i] != "NaN" and SensorReading.ranges[i] <= SensorReading.range_max and SensorReading.ranges[i] >= SensorReading.range_min:
            #Compute the end point of the ray 
            ray_angle = SensorReading.start_angle + (SensorReading.angle_increment*i) + Robotpose[2] + (3*math.pi/4)

            x = SensorReading.ranges[i]*math.cos(ray_angle)+Robotpose[0]   
            y = SensorReading.ranges[i]*math.sin(ray_angle)+Robotpose[1]

            
            # print("RayLength : " + str(SensorReading.ranges[400]) + " X: " + str(int((x+50)/0.02)) + " Y: " + str(int((y+50)/0.02)) + " Ray Angle: " + str(ray_angle))

            #Mark the cell as occupied
            cells[int((x+50)/0.02),int((y+50)/0.02)] = 100
    
            #mark the cells that are in the sensor range as free if not occupied
            step_size = SensorReading.ranges[i]/50
            for j in range(50):
                x = step_size*j*math.cos(ray_angle)+Robotpose[0]
                y = step_size*j*math.sin(ray_angle)+Robotpose[1]
                if cells[int((x+50)/0.02),int((y+50)/0.02)] != 100:
                    cells[int((x+50)/0.02),int((y+50)/0.02)] = 0 





def onDataRecived(msg , pub):
    map = OccupancyGrid() #map data to be published

    # print(msg.pose.pose.position)

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
    #intializing Node
    rospy.init_node('Mapping')
    
    #publish The map data to the topic map_data
    pub = rospy.Publisher("map_data",OccupancyGrid,queue_size= 10)

    #subscribe to the aligned Sensor Readings topic
    rospy.Subscriber(name = "/sensor_readings" ,
                    data_class= Readings ,  
                    callback= onDataRecived ,
                    callback_args=(pub),
                    queue_size= 1)

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass