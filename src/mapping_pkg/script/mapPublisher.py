from mapper import Mapper
from nav_msgs.msg import OccupancyGrid
import rospy



class MapPublisher(Mapper):
    def __init__(self, publishTopic, RearLaserTransformMatrix, FrontLaserTransformMatrix, \
        mapMetaData, referenceFrame):
        
        #create a publisher to publish the map
        self.pub = rospy.Publisher(publishTopic,OccupancyGrid,queue_size= 1)

        #call the super class constructor
        super().__init__(RearLaserTransformMatrix, FrontLaserTransformMatrix ,mapMetaData , referenceFrame)

    def onDataRecived(self, map):
        #publish the map
        # self.pub.publish(super().onDataRecived(pose,msg))
        self.pub.publish(map)