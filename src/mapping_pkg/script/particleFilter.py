from mapper import Mapper
import numpy as np

class particleFilter:
    def __init__(self , n_particles , motionModel , sensorModel):
        self.n_particles = n_particles
        self.motionModel = motionModel
        self.sensorModel = sensorModel
        self.particles = []
        self.weights = []
        self.pose = None
        self.mapper = None


    def initParticles(self , Current_odom_pose):
        for _ in range(self.n_particles):
            mapper = Mapper()
            pose = mapper.generateRandomPose()
            weight = 1 / self.n_particles
            self.particles.append(particle(pose , weight , mapper , Current_odom_pose))
            self.weights.append(weight)


    def resampleParticles(self):
        new_particles = []
        new_weights = []
        for _ in range(self.n_particles):
            index = np.random.choice(range(self.n_particles) , p = self.weights)
            new_particles.append(self.particles[index])
            new_weights.append(self.weights[index])
        self.particles = new_particles
        self.weights = new_weights


    def updateParticles(self , msg):
        odom = [msg.pose.pose.position.x , msg.pose.pose.position.y , msg.pose.pose.orientation.z]
        laserScanFront = msg.ranges_front
        laserScanRear = msg.ranges_rear
        accum = 0
        for i in range(self.n_particles):
            prev_pose = self.particles[i].pose
            curr_odom_pose = self.particles[i].Curr_odom_pose
            prev_odom_pose = self.particles[i].prev_odom_pose
            self.particles[i].updatePose(self.motionModel.SampleMotionModel(prev_odom_pose , curr_odom_pose , prev_pose))
            self.particles[i].updateOdom(odom)
            
            frontScannerCorrection = self.sensorModel.p_z_given_x_m(self.particles[i].pose , laserScanFront , self.particles[i].mapper)
            rearScannerCorrection = self.sensorModel.p_z_given_x_m(self.particles[i].pose , laserScanRear , self.particles[i].mapper , False)
            self.particles[i].updateWeight(frontScannerCorrection + rearScannerCorrection)
            
            self.particles[i].mapper.computeOccupancy(msg)

            accum += self.particles[i].weight
            self.weights[i] = self.particles[i].weight
        self.weights = [w / accum for w in self.weights]
        self.pose = max(self.particles , key = lambda p : p.weight).pose
        self.mapper = max(self.particles , key = lambda p : p.weight).mapper






#Helper class for particle filter
class particle:
    def __init__(self , pose , weight , mapper , Current_odom_pose):
        self.pose = pose
        self.prev_odom_pose = pose 
        self.Curr_odom_pose = Current_odom_pose
        self.weight = weight
        self.mapper = mapper

    def updatePose(self , pose):
        self.pose = pose
    
    def updateOdom(self , odom):
        self.prev_odom_pose = self.Curr_odom_pose
        self.Curr_odom_pose = odom

    def updateWeight(self , weight):
        self.weight = weight
    
    