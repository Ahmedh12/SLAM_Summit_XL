#!/usr/bin/env python3
import numpy as np
from utils import Transformation , normalize_angle , get_quaternion_from_euler
import math
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

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
    def __init__(self  ,RearLaserTransformMatrix , FrontLaserTransformMatrix , mapMetaData):

        self.cells = np.ones((mapMetaData.height,mapMetaData.width),dtype=np.int8)
        self.cells = self.cells * -1 # -1 means unknown, 0 means free, 100 means occupied
        self.FLTM = FrontLaserTransformMatrix
        self.RLTM = RearLaserTransformMatrix
        self.mapMetaData = mapMetaData
        self.RT = np.eye(4)

    def getGlobalCoords(self,x,y,transform , toOdomTransform = None):
        temp = np.array([x,y,1,1])
        temp = np.matmul(transform,temp)
        if toOdomTransform is None:
            temp = np.matmul(self.RT,temp)
        else:
            temp = np.matmul(toOdomTransform,temp)

        x = temp[1]
        y = temp[0]
        return x,y
    
    def getRayCoords(self , range , angle , transform , toOdomTransform = None):
        x = range*math.cos(angle)   
        y = range*math.sin(angle)              
        if toOdomTransform is None:
            x,y = self.getGlobalCoords(x,y,transform)
        else:
            x,y = self.getGlobalCoords(x,y,transform,toOdomTransform)
            
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
                    self.cells[x,y] = 100
                        # if self.cells[x,y] == -1:
                        #     self.cells[x,y] = (0.9/0.1)*10
                        # else:
                        #     self.cells[x,y] = ((0.9/0.1)*(self.cells[x,y]/10))*10
                
                #Mark the cells that are in the sensor range as free if not occupied
                step_size = ranges[i] * self.mapMetaData.resolution
                for j in range(int(1/self.mapMetaData.resolution)-1):
                    x,y = self.getRayCoords(step_size*j,ray_angle,transform)
                    if self.inMap(x,y):
                        self.cells[x,y] = max(0,self.cells[x,y])
                        # if self.cells[x,y] == -1:
                        #     self.cells[x,y] = (0.1/0.9)*10
                        # else:
                        #     self.cells[x,y] = ((0.1/0.9)*(self.cells[x,y]/10))*10
        
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
    
    def ray_casting(self, x_t , ray_angle , max_range , transform):
        '''
        Retrurns the length of the ray casted at the given pose and ray index
        according to the map constructed so far
        x_t : the pose of the robot at time t in the odom frame
        ray_angle : the angle of the ray in its frame
        '''
        #Get the robot odom transformation matrix
        pos = [x_t.position.x,x_t.position.y,x_t.position.z]
        rot = [x_t.orientation.x,x_t.orientation.y, x_t.orientation.z,x_t.orientation.w]
        RT = Transformation(parentFrame="" , 
                            childFrame="" , 
                            pos = pos , rot= rot).transformationMatrix()
        
        ray_angle = normalize_angle(ray_angle)

        #cast the ray
        step_size = max_range * self.mapMetaData.resolution
        for j in range(int(1/self.mapMetaData.resolution)):
            x,y = self.getRayCoords(step_size*j,ray_angle,transform,RT)
            y_map = int((x_t.position.x - self.mapMetaData.origin.position.x) / self.mapMetaData.resolution)
            x_map = int((x_t.position.y - self.mapMetaData.origin.position.y) / self.mapMetaData.resolution)

            if self.inMap(x,y) and self.cells[x,y] == 100:
                # print("hit")
                # print("pose in map",str(x_map),str(y_map),sep=",")
                # print("grid",str(x),str(y) , sep=",")
                break

        #Compute the length of the ray                
        ray_length = math.sqrt((x-x_map)**2 + (y-y_map)**2)
        
        return ray_length * self.mapMetaData.resolution

    def generateRandomPose(self):
        '''
        Generates a random pose in the map
        '''
        x_lower = self.mapMetaData.origin.position.x
        x_upper = (self.mapMetaData.origin.position.x)  + self.mapMetaData.width * self.mapMetaData.resolution
        y_lower = self.mapMetaData.origin.position.y
        y_upper = (self.mapMetaData.origin.position.y)  + self.mapMetaData.height * self.mapMetaData.resolution
        x = np.random.uniform(x_lower/5,x_upper/5)
        y = np.random.uniform(y_lower/5,y_upper/5)
        theta = np.random.uniform(-math.pi,math.pi)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,theta)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        

        return pose

    def onDataRecived(self,pose,msg):
        map = OccupancyGrid()
        map.header.frame_id = self.referenceFrame
        map.info = self.mapMetaData
        robot_odom = pose
        pos = [robot_odom.position.x,robot_odom.position.y,robot_odom.position.z]
        rot = [robot_odom.orientation.x,robot_odom.orientation.y, robot_odom.orientation.z,robot_odom.orientation.w]
        self.RT = Transformation(parentFrame="" , childFrame="" , pos = pos , rot= rot).transformationMatrix() 
        self.computeOccupancy(msg)
        map.data = tuple(self.cells.flatten())
        return map