#!/usr/bin/env python3
import rospy
import numpy as np
from mapping_pkg.msg import Readings
from nav_msgs.msg import OccupancyGrid
from utils import Transformation
import math


'''
Assumptions:
1- Assume msg.pose is the pose of the robot at the time of the sensor reading relative to the real world frame
2- Assume the Sensor reading are very accurate , so the inverse sensor model is not needed
3- Assume the prior is uniform
'''

'''
Observations:
1-The Odom frame drifts a little bit , so the mappping transform is not 100% accurate
'''

class Mapper:
    def __init__(self  ,RearLaserTransformMatrix , FrontLaserTransformMatrix ,\
         mapMetaData , referenceFrame , sensorTopic):

        self.cells = np.ones((mapMetaData.height,mapMetaData.width),dtype=np.int8)
        self.cells = self.cells * -1 # -1 means unknown, 0 means free, 100 means occupied
        self.referenceFrame = referenceFrame
        self.FLTM = FrontLaserTransformMatrix
        self.RLTM = RearLaserTransformMatrix
        self.mapMetaData = mapMetaData

        #subscribe to the aligned Sensor Readings topic
        rospy.Subscriber(name = sensorTopic ,
                        data_class= Readings ,  
                        callback= self._onDataRecived,
                        queue_size= 1)

    def getGlobalCoords(self,x,y,transform):
        temp = np.array([x,y,1,1])
        temp = np.matmul(transform,temp)
        temp = np.matmul(self.RT,temp)

        x = temp[1]
        y = temp[0]
        return x,y
    
    def getRayCoords(self , range , angle , transform):
        x = range*math.cos(angle)   
        y = range*math.sin(angle)              
        x,y = self.getGlobalCoords(x,y,transform)
        offset_x = self.mapMetaData.origin.position.x
        offset_y = self.mapMetaData.origin.position.y
        resolution = self.mapMetaData.resolution

        x = int((x - offset_x) / resolution)
        y = int((y - offset_y) / resolution)
        return x,y

    def inMap(self,x,y):
        offset_x = self.mapMetaData.origin.position.x
        offset_y = self.mapMetaData.origin.position.y
        bound_x = self.mapMetaData.width + offset_x
        bound_y = self.mapMetaData.height + offset_y

        if x>= offset_x and x<= bound_x and y>= offset_y and y<= bound_y:
            return True
        else:
            return False
    
    def followRays(self,ranges , range_max , range_min , start_angle ,\
         angle_increment , transform):
        #Check for ray end points to mark cells as occupied
        for i in range(len(ranges)):
            if ranges[i] != "NaN" and ranges[i] <= range_max and ranges[i] >= range_min:
                #Compute the end point of the ray 
                ray_angle = start_angle + (angle_increment*i)
                x,y = self.getRayCoords(ranges[i],ray_angle,transform)
                #Mark the cell as occupied
                if self.inMap(x,y):
                        if self.cells[x,y] == -1:
                            self.cells[x,y] = (0.9/0.1)*10
                        else:
                            self.cells[x,y] = ((0.9/0.1)*(self.cells[x,y]/10))*10
                
                #Mark the cells that are in the sensor range as free if not occupied
                step_size = ranges[i] * self.mapMetaData.resolution
                for j in range(int(1/self.mapMetaData.resolution)-1):
                    x,y = self.getRayCoords(step_size*j,ray_angle,transform)
                    if self.inMap(x,y):
                        if self.cells[x,y] == -1:
                            self.cells[x,y] = (0.1/0.9)*10
                        else:
                            self.cells[x,y] = ((0.1/0.9)*(self.cells[x,y]/10))*10
        
    def computeOccupancy(self,SensorReading):
        self.followRays(SensorReading.ranges_front,
                        SensorReading.range_max,
                        SensorReading.range_min,
                        SensorReading.start_angle_front,
                        SensorReading.angle_increment_front, 
                        self.FLTM)

        self.followRays(SensorReading.ranges_rear,
                        SensorReading.range_max,
                        SensorReading.range_min,
                        SensorReading.start_angle_rear,
                        SensorReading.angle_increment_rear, 
                        self.RLTM)

    def _onDataRecived(self,msg):
        map = OccupancyGrid()
        map.header.frame_id = self.referenceFrame
        map.info = self.mapMetaData
        robot_odom = msg.pose.pose
        #Get the robot odom transformation matrix
        pos = [robot_odom.position.x,robot_odom.position.y,robot_odom.position.z]
        rot = [robot_odom.orientation.x,robot_odom.orientation.y, robot_odom.orientation.z,robot_odom.orientation.w]
        self.RT = Transformation(parentFrame="" , childFrame="" , pos = pos , rot= rot).transformationMatrix() 
        
        self.computeOccupancy(msg)
        map.data = tuple(self.cells.flatten())
        self.pub.publish(map)