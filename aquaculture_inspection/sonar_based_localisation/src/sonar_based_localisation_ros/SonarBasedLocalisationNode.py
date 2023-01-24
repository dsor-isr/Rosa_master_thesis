#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from sonar_based_localisation_algorithms.SonarBasedLocalisationAlgorithm import sonarBasedNetCenterDetection
from std_msgs.msg import Int8, Bool
from sensor_msgs.msg import Image
from auv_msgs.msg import NavigationStatus
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class SonarBasedLocalisationNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('sonar_based_localisation_node')

        
        """
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        """
        self.loadParams()
        
        # Instanciate the Center Detector object
        self.sonar_detector = sonarBasedNetCenterDetection(self.sonar_range_, self.net_radius_)
        
      
        self.initializeSubscribers()
        self.initializePublishers()
        


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        self.utm_pos_ = rospy.get_param('~UTM_POS')
        self.real_center_ = rospy.get_param('~POS_center')
        self.net_radius_ = rospy.get_param('~Net_radius')
        self.sonar_range_ = rospy.get_param('~Sonar_range')
        self.sse_threshold_ = rospy.get_param('~SSE_threshold')
        self.d_outlier_ = rospy.get_param('~Distance_of_outliers')
        self.nmin_points_ = rospy.get_param('~nmin_points')
        self.bridge_ = CvBridge()

        self.vehicle_ = rospy.get_param('~Vehicle')
        self.font_ = cv.FONT_HERSHEY_PLAIN
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber(self.vehicle_ + '/multibeam/sonar_image_topic', Image, self.image_update_callback, queue_size=5)
        rospy.Subscriber(self.vehicle_+'/nav/filter/state', NavigationStatus, self.State_callback, queue_size=1)
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for SonarBasedLocalisationNode')
        self.detection_image_pub_ = rospy.Publisher('/detection/object/detection_visualization/', Image, queue_size=1)
        self
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)



    def image_update_callback(self, data):
        try:
            cv_image = self.bridge_.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        # Compute Center of the Fishing Net in the Sonar Image
        detected_flag, xc, yc, img, binary_img = self.sonar_detector.detect_circle(cv_image, self.sse_threshold_, self.d_outlier_, self.nmin_points_)
        
        # Compute Real Center in Body Frame
        if detected_flag:
            height = img.shape[0]
            width = img.shape[1]
            pos_center_inertial, sonar_pos, center_pos_body = self.computeCenter(xc, yc, height, width, img)
        else:
            img_msg = self.bridge_.cv2_to_imgmsg(cv_image, "bgr8")
            self.detection_image_pub_.publish(img_msg)


    '''
        Function for computing the yaw reference to steer the vehicle to be facing towards the center of the net
    '''
    def computeDesiredYaw(self):
        pass

    def computeCenter(self, xc, yc, heigth, width, img):

        sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, heigth)
        xp = sonar_pos[0]
        yp = sonar_pos[1]
        # Note that coordinates in body frame are in NED
        xc_body = (yp-yc) * self.sonar_range_/heigth
        yc_body = (xc-xp) * self.sonar_range_/heigth

        center_pos_body = np.array([[xc_body], [yc_body]]) # Position Center in Body Frame
        pos_vehicle_intertial = np.array([[self.x], [self.y]])

        print("CENTER BODY: " + str(center_pos_body))
        print("POS Vehicle: " + str(pos_vehicle_intertial))
        R = self.computeRotationMatrix()
        print("R: " + str(R))
        print("Multiplication: " + str(np.dot(R, center_pos_body)))
        pos_center_intertial = np.dot(R, center_pos_body) + pos_vehicle_intertial
        print("POS CALCULADA: " + str(pos_center_intertial))
        # Show the computed circle
        self.pubImageWithCircle(xc, yc, pos_center_intertial, sonar_pos, center_pos_body, img)
        return pos_center_intertial, sonar_pos, center_pos_body

    def computeRotationMatrix(self):
        yaw_radians = np.deg2rad(self.yaw)
        R = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians)], [np.sin(yaw_radians), np.cos(yaw_radians)]])
        return R

        
    '''
        Display Computed Circle info in the Sonar image
    '''
    def pubImageWithCircle(self, xc, yc, pos_center_inertial, sonar_pos, center_pos_body, img):
        color = (255, 0, 0)
        radius_px = self.sonar_detector.computeRadiusPxValue(img)
        cv.circle(img, (int(xc), int(yc)), int(radius_px), color, 2)
        cv.circle(img, (int(xc), int(yc)), 2, color, 2)

        text = "Depth(meters): {:.2f}".format(self.depth)
        cv.putText(img, text, (10, 70), self.font_, 1, (255,255,255), 1)
        
        text = "Center in Image(pixels) [x, y]: [" + str(int(xc)) + "," + str(int(yc)) + "]"
        cv.putText(img, text, (10, 90), self.font_, 1, (255,255,255), 1)

        text = "Sonar POS (pixels) [x, y]: [" + str(sonar_pos[0]) + "," + str(sonar_pos[1]) + "]"
        cv.putText(img, text, (0, 110), self.font_, 1, (255,255,255), 1)

        text = "Center Pos Body [x, y]: [" + str(center_pos_body[0]) + "," + str(center_pos_body[1]) + "]"
        cv.putText(img, text, (0, 130), self.font_, 1, (255,255,255), 1)

        text = "Pos Vehicle [x, y]: [" + str(self.x) + "," + str(self.y) + "]"
        cv.putText(img, text, (0, 150), self.font_, 1, (255,255,255), 1)

        text = "Center Inertial Estimated (NED) [x, y]: [" + str(pos_center_inertial[0]) + "," + str(pos_center_inertial[1]) + "]"
        cv.putText(img, text, (10, 170), self.font_, 1, (255,0,255), 1)

        text = "Center Inertial REAL (NED) [x, y]: [" + str(self.real_center_[0]) + "," + str(self.real_center_[1]) + "]"
        cv.putText(img, text, (10, 190), self.font_, 1, (255,0,255), 1)

        text = "Yaw(Graus)"+ str(self.yaw)
        cv.putText(img, text, (10, 200), self.font_, 1, (255,0,255), 1)

        img_msg = self.bridge_.cv2_to_imgmsg(img, "bgr8")
        self.detection_image_pub_.publish(img_msg)
        

    def State_callback(self, data):
        self.surge = data.body_velocity.x
        self.sway = data.body_velocity.y
        self.yaw = data.orientation.z

        self.x = data.position.north - self.utm_pos_[0]
        self.y = data.position.east - self.utm_pos_[1]
        self.depth = data.position.depth

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
            
    

def main():

    sonar_based_localisation = SonarBasedLocalisationNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
