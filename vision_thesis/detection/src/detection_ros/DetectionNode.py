#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from detection.src.detection_algorithms.ibvs import VisualServoing
from std_msgs.msg import Int8, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from auv_msgs.msg import NavigationStatus
from uuv_sensor_ros_plugins_msgs.msg import DVL

class DetectionNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('detection_node')

        
        """
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        """

        

        """
        @.@ Dirty work of declaring subscribers, publishers and load parameters 
        """
        self.loadParams()
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        self.vehicle = rospy.get_param('~Vehicle')
        self.bridge = CvBridge()
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for DetectionNode')
        rospy.Subscriber(self.vehicle+'/bluerov/camera/camera_image', Image, self.image_update_callback, queue_size=1)

    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for DetectionNode')


    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()


    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        # REMOVE pass and do your magic
        pass

    def image_update_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bfr8")
        except CvBridgeError as e:
            raise e

            


def main():

    detection = DetectionNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
