from mappingWithKnownPoses import Mapper
from nav_msgs.msg import OccupancyGrid
import rospy


class MapPublisher(Mapper):
    def __init__(self, publishTopic, RearLaserTransformMatrix, FrontLaserTransformMatrix, \
        mapMetaData, referenceFrame,sensorTopic):
        super().__init__(RearLaserTransformMatrix, FrontLaserTransformMatrix,\
            mapMetaData, referenceFrame, sensorTopic)

        #intializing Node
        rospy.init_node('Mapping')
        
        #create a publisher to publish the map
        self.pub = rospy.Publisher(publishTopic,OccupancyGrid,queue_size= 1)
        
