from mapper import Mapper
from nav_msgs.msg import OccupancyGrid
from mapping_pkg.msg import Readings
from utils import Transformation
import rospy



class MapPublisher(Mapper):
    def __init__(self, publishTopic, RearLaserTransformMatrix, FrontLaserTransformMatrix, \
        mapMetaData, referenceFrame):

        #set the reference frame
        self.referenceFrame = referenceFrame
        
        #create a publisher to publish the map
        self.pub = rospy.Publisher(publishTopic,OccupancyGrid,queue_size= 1)

        #call the super class constructor
        super().__init__(RearLaserTransformMatrix, FrontLaserTransformMatrix ,mapMetaData)

    def onDataRecived(self, pose,msg):
        #publish the map
        self.pub.publish(super().onDataRecived(pose,msg))