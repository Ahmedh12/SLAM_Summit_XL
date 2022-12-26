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
        self.maxWeight = 0
        self.map = None
        self.mapPublisher =  MapPublisher(publishTopic = "map_data",
                    RearLaserTransformMatrix = RLTM,
                    FrontLaserTransformMatrix = FLTM,
                    mapMetaData = mapMetaData,
                    referenceFrame = "robot_map")


    def initParticles(self , msg):
        for _ in range(self.n_particles):
            mapper = Mapper(RLTM, FLTM , mapMetaData , "robot_map")
            curr_odom_pose = msg.pose.pose
            pose = curr_odom_pose #mapper.generateRandomPose()
            weight = 1 / self.n_particles
            self.particles.append(particle(pose , weight , mapper , curr_odom_pose))
            self.weights.append(weight)
        
        self.pose = self.particles[0].pose
        self.particles[0].mapper.onDataRecived(self.particles[0].pose , msg)
        for i in range(1 , self.n_particles):
            self.particles[i].mapper.map = self.particles[0].mapper.map
        self.map = self.particles[0].mapper.map
        
        # self.mapPublisher.cells = self.particles[0].mapper.cells

    def updateParticles(self , msg):
        odom = msg.pose.pose
        laserScanFront = msg.ranges_front
        laserScanRear = msg.ranges_rear
        accum = 0

        for i in range(self.n_particles):
            print("particle: " , i)
            print(self.particles[i].pose , self.particles[i].weight , sep=",")
            prev_pose = self.particles[i].pose
            curr_odom_pose = odom
            prev_odom_pose = self.particles[i].curr_odom_pose

            self.particles[i].updatePose(self.motionModel.sampleMotionModelOdom(prev_odom_pose , curr_odom_pose , prev_pose))
            # self.particles[i].updatePose(self.motionModel.sampleMotionModelVelocity([msg.twist.twist.linear.x , msg.twist.twist.angular.z] , msg.header.stamp.secs , prev_pose))
            self.particles[i].updateOdom(odom)
            frontScannerCorrection = self.sensorModel.p_z_given_x_m(laserScanFront, self.particles[i].pose  , self.particles[i].mapper)
            rearScannerCorrection = self.sensorModel.p_z_given_x_m(laserScanRear, self.particles[i].pose  , self.particles[i].mapper , False)
            self.particles[i].updateWeight(frontScannerCorrection + rearScannerCorrection)

            self.particles[i].mapper.onDataRecived(self.particles[i].pose,msg)

            accum += self.particles[i].weight
            self.weights[i] = self.particles[i].weight

        #normalize weights
        self.weights = [w / accum for w in self.weights]

        #select the best particle and update the map with it
        bestParticle = max(self.particles , key = lambda p : p.weight)
        self.maxWeight = bestParticle.weight
        self.map = bestParticle.mapper.map
        self.pose = bestParticle.pose

        #update all particles with the best map
        for i in range(self.n_particles):
            self.particles[i].mapper.map = self.map


    def resampleParticles(self):
        new_particles = []
        new_weights = []
        for _ in range(self.n_particles):
            index = np.random.choice(range(self.n_particles) , p = self.weights)
            new_particles.append(self.particles[index])
            new_weights.append(self.weights[index])
        self.particles = new_particles
        self.weights = new_weights

    
    def printParticles(self):
        for i in range(self.n_particles):
            print("particle: " , i)
            print(self.particles[i].pose , self.particles[i].weight , sep=",")
        print("best pose: " , str(self.pose) , "weight: " , str(max(self.particles , key = lambda p : p.weight).weight))


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
    
    