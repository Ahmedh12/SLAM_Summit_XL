from mappingWithKnownPoses import Mapper
from nav_msgs.msg import OccupancyGrid
from mapping_pkg.msg import Readings
from utils import Transformation
import rospy



class MapPublisher(Mapper):
    def __init__(self, publishTopic, RearLaserTransformMatrix, FrontLaserTransformMatrix, \
        mapMetaData, referenceFrame,sensorTopic):

        #set the reference frame
        self.referenceFrame = referenceFrame

        #intializing Node
        rospy.init_node('Mapping')
        
        #create a publisher to publish the map
        self.pub = rospy.Publisher(publishTopic,OccupancyGrid,queue_size= 1)

        #create a subscriber to subscribe to the sensor readings
        rospy.Subscriber(name = sensorTopic, 
                         data_class= Readings, 
                         callback= self._onDataRecived, 
                         queue_size= 1)


        #call the super class constructor
        super().__init__(RearLaserTransformMatrix, FrontLaserTransformMatrix ,mapMetaData)
        

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