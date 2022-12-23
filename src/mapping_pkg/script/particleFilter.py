from mapper import Mapper
import numpy as np

from constants import mapMetaData , RLTM , FLTM
from mapPublisher import MapPublisher


class particleFilter:
    def __init__(self , n_particles , motionModel , sensorModel):
        self.n_particles = n_particles
        self.motionModel = motionModel
        self.sensorModel = sensorModel
        self.particles = []
        self.weights = []
        self.pose = None
        self.mapPublisher =  MapPublisher(publishTopic = "map_data",
                    RearLaserTransformMatrix = RLTM,
                    FrontLaserTransformMatrix = FLTM,
                    mapMetaData = mapMetaData,
                    referenceFrame = "robot_map")


    def initParticles(self , msg):
        for _ in range(self.n_particles):
            mapper = Mapper(RLTM, FLTM , mapMetaData)
            pose = mapper.generateRandomPose()
            weight = 1 / self.n_particles
            curr_odom_pose = msg.pose.pose
            self.particles.append(particle(pose , weight , mapper , curr_odom_pose))
            self.weights.append(weight)
        
        self.pose = self.particles[0].pose
        self.particles[0].mapper.computeOccupancy(msg)
        # self.mapPublisher.cells = self.particles[0].mapper.cells

    def updateParticles(self , msg):
        odom = msg.pose.pose
        laserScanFront = msg.ranges_front
        laserScanRear = msg.ranges_rear
        accum = 0

        for i in range(self.n_particles):
            prev_pose = self.particles[i].pose
            curr_odom_pose = self.particles[i].curr_odom_pose
            prev_odom_pose = self.particles[i].prev_odom_pose
            self.particles[i].updatePose(self.motionModel.SampleMotionModel(prev_odom_pose , curr_odom_pose , prev_pose))
            self.particles[i].updateOdom(odom)
            frontScannerCorrection = self.sensorModel.p_z_given_x_m(laserScanFront, self.particles[i].pose  , self.particles[i].mapper)
            rearScannerCorrection = self.sensorModel.p_z_given_x_m(laserScanRear, self.particles[i].pose  , self.particles[i].mapper , False)
            self.particles[i].updateWeight(frontScannerCorrection + rearScannerCorrection)
            
            #In The end of the loop we will have the best pose , 
            # so we will compute the map for the best pose
            # only to reduce the computation time
            # self.particles[i].mapper.computeOccupancy(msg)

            accum += self.particles[i].weight
            self.weights[i] = self.particles[i].weight
        #normalize weights
        self.weights = [w / accum for w in self.weights]
        self.pose = max(self.particles , key = lambda p : p.weight).pose
        # print("best pose: " , str(self.pose) , "weight: " , str(max(self.particles , key = lambda p : p.weight).weight))
        #TODO:
        self.pose = self.particles[0].curr_odom_pose
        # self.mapPublisher.cells = max(self.particles , key = lambda p : p.weight).mapper.cells


    def resampleParticles(self):
        new_particles = []
        new_weights = []
        for _ in range(self.n_particles):
            index = np.random.choice(range(self.n_particles) , p = self.weights)
            new_particles.append(self.particles[index])
            new_weights.append(self.weights[index])
        self.particles = new_particles
        self.weights = new_weights



#Helper class for particle filter
class particle:
    def __init__(self , pose , weight , mapper , Current_odom_pose):
        self.pose = pose
        self.prev_odom_pose = pose 
        self.curr_odom_pose = Current_odom_pose
        self.weight = weight
        self.mapper = mapper

    def updatePose(self , pose):
        self.pose = pose
    
    def updateOdom(self , odom):
        self.prev_odom_pose = self.curr_odom_pose
        self.curr_odom_pose = odom

    def updateWeight(self , weight):
        self.weight = weight
    
    