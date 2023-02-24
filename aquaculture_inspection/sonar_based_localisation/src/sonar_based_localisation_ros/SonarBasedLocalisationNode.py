#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from sonar_based_localisation_algorithms.SonarBasedLocalisationAlgorithm import sonarBasedNetCenterDetection
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from auv_msgs.msg import NavigationStatus, NED
from sonar_based_localisation.msg import DetectionResults
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
        self.sonar_detector = sonarBasedNetCenterDetection(self.sonar_range_, self.net_radius_, \
                            self.dist_critical_, self.dist_between_posts_)
        
      
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()
        self.initializeTimer()


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        self.utm_pos_inertial_ = rospy.get_param('~utm_pos_inertial')
        self.real_center_ = rospy.get_param('~pos_center')
        self.net_radius_ = rospy.get_param('~net_radius')
        self.sonar_range_ = rospy.get_param('~sonar_range')
        self.dist_between_posts_ = rospy.get_param('~dist_between_posts')

        self.dist_critical_ = rospy.get_param('~dist_critical')
        
        self.d_outlier_ = rospy.get_param('~distance_of_outliers')
        self.nmin_points_ = rospy.get_param('~nmin_points')
        self.bridge_ = CvBridge()
        self.starting_point_ = None

        self.vehicle_ = rospy.get_param('~Vehicle')
        self.font_ = cv.FONT_HERSHEY_PLAIN

        self.last_center_ = None
        self.counter_outlier_ = 0
        self.estimation_counter_ = 0
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber(self.vehicle_ + '/multibeam/sonar_image_topic', Image, self.image_update_callback, queue_size=5)
        rospy.Subscriber(self.vehicle_+'/nav/filter/state', NavigationStatus, self.state_callback, queue_size=1)
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for SonarBasedLocalisationNode')
        self.detection_image_pub_ = rospy.Publisher('/detection/object/detection_visualization/', Image, queue_size=1)
        self.detection_info_pub_ = rospy.Publisher('/detection/object/detection_results', DetectionResults, queue_size=1)
        self.yaw_desired_pub_ = rospy.Publisher('/yaw_check', Float64, queue_size=1)
        self.detection_flag_pub_ = rospy.Publisher('/detection/flag', Bool, queue_size=1)
        self.position_wrapped_pub_ = rospy.Publisher('/position_wrapped', NED, queue_size=1)


    def initializeServices(self):
        rospy.loginfo('Initializing Services for SonarBasedLocalisationNode')
        
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    def image_update_callback(self, data):
        try:
            self.cv_image_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
            print("Image Received")
        except CvBridgeError as e:
            raise e
        
        height = self.cv_image_.shape[0]
        width = self.cv_image_.shape[1]
        if self.starting_point_ is None:
            sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, height)
            self.starting_point_ = self.computeInitialPos(height, sonar_pos)
        sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, height)
        self.starting_point_ = self.computeInitialPos(height, sonar_pos)

    
    """
        Function to check if the starting Point is valid in case of a miscalculation
    """
    def checkStartingPointValidity(self, img_height):
        #In case the estimation of the center is behind the sonar (Impossible)
        if self.starting_point_[1] >= img_height:
            self.starting_point_ = None
        
    '''
        Function for computing the yaw reference to steer the vehicle to be facing towards the center of the net
    '''
    def computeDesiredYaw(self, center_pos_pixels, sonar_pos, yaw):
        x_c = center_pos_pixels[0]
        y_c = center_pos_pixels[1]
        x_sonar = sonar_pos[0]
        y_sonar = sonar_pos[1]
        
        atan_term = np.arctan2(y_sonar - y_c, -(x_sonar - x_c))*180/np.pi
        yaw_rel = 90 - atan_term
        # yaw_error = yaw_ref - yaw
        yaw_desired = yaw + yaw_rel
        return yaw_desired, atan_term

    
    """
        Function to compute the Initial Condition for the Least Squares Method
        Which is the Projection of the real center coordinates in Sonar Image pixels
    """
    def computeInitialPos(self, img_height, sonar_pos_pixels):
        yaw_rad = np.deg2rad(self.yaw_)
        R_I2B = np.array([[np.cos(yaw_rad), np.sin(yaw_rad)],[-np.sin(yaw_rad), np.cos(yaw_rad)]])

        pos_vehicle = np.array([[self.x_],[self.y_]])
        center_inertial = np.array([[self.real_center_[0]], [self.real_center_[1]]])
        center_pos_body = np.dot(R_I2B, (center_inertial - pos_vehicle))
        # the sonar coordinates array and the center in the body frames are switched, because the body frame is in NED notation
        xc_px = center_pos_body[1] * img_height/self.sonar_range_ + sonar_pos_pixels[0]
        # This one has a minus, cause the y-axis in the image points downwards
        yc_px = sonar_pos_pixels[1] - center_pos_body[0] * img_height/self.sonar_range_
        
        return np.array([xc_px, yc_px])

    def computeCenter(self, xc, yc, heigth, width):
        sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, heigth)
        xp = sonar_pos[0]
        yp = sonar_pos[1]
        # Note that coordinates in body frame are in NED
        xc_body = (yp-yc) * self.sonar_range_/heigth
        yc_body = (xc-xp) * self.sonar_range_/heigth

        center_pos_body = np.array([[xc_body], [yc_body]]) # Position Center in Body Frame
        pos_vehicle_inertial = np.array([[self.x_], [self.y_]])

        R = self.computeRotationMatrix()
                
        pos_center_inertial = np.dot(R, center_pos_body) + pos_vehicle_inertial
        
        return pos_center_inertial, sonar_pos, center_pos_body

    def computeRotationMatrix(self):
        yaw_radians = np.deg2rad(self.yaw_)
        R = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians)], [np.sin(yaw_radians), np.cos(yaw_radians)]])
        return R

        
    '''
        Display Computed Circle info in the Sonar image
    '''
    def pubImageWithCircle(self, xc, yc, pos_center_inertial, sonar_pos, center_pos_body, img, distance_net, binary_img, not_outlier_flag):
        color = (255, 0, 0)
        height = img.shape[0]
        
        img2 = np.zeros_like(img)
        
        img2[:,:,0] = binary_img
        img2[:,:,1] = binary_img
        img2[:,:,2] = binary_img

        img = img2

        # Draw used points
        n_data = self.point_coordinates_.shape[0]
        for i in range(0, n_data):
            cv.circle(img, (int(self.point_coordinates_[i, 0]), int(self.point_coordinates_[i, 1])), int(1), (0,255,0), 2)    

        radius_px = self.sonar_detector.convertMeter2Pixels(height, self.net_radius_)
        #show regressed circle
        if not_outlier_flag:
            cv.circle(img, (int(xc), int(yc)), int(radius_px), color, 2)
        else:
            cv.circle(img, (int(xc), int(yc)), int(radius_px), (255, 255, 255), 2)
        # center of the circle
        cv.circle(img, (int(xc), int(yc)), 2, color, 2)
        
        centroid_post = self.sonar_detector.getCentroidPost()
        if centroid_post is not None:
            dist_between_post_px = self.sonar_detector.convertMeter2Pixels(height, self.dist_between_posts_)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px), (0,0,255), 2)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px-10), (0,0,255), 2)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px+10), (0,0,255), 2)
        
        second_post = self.sonar_detector.getSecondCentroidPost()
        if second_post is not None:
            cv.circle(img, (int(second_post[0]), int(second_post[1])), int(40), (255,0,127), 2)

        text = "Depth(meters): {:.2f}".format(self.depth_)
        cv.putText(img, text, (0, 70), self.font_, 1, (255,255,255), 1)
        
        text = "Center in Image(pixels) [x, y]: [" + str(int(xc)) + "," + str(int(yc)) + "]"
        cv.putText(img, text, (0, 90), self.font_, 1, (255,255,255), 1)

        text = "Sonar POS (pixels) [x, y]: [" + str(sonar_pos[0]) + "," + str(sonar_pos[1]) + "]"
        cv.putText(img, text, (0, 110), self.font_, 1, (255,255,255), 1)

        text = "Center Pos Body [x, y]: [" + str(center_pos_body[0]) + "," + str(center_pos_body[1]) + "]"
        cv.putText(img, text, (0, 130), self.font_, 1, (255,255,255), 1)

        text = "Pos Vehicle [x, y]: [" + str(self.x_) + "," + str(self.y_) + "]"
        cv.putText(img, text, (0, 150), self.font_, 1, (255,255,255), 1)

        text = "Center Inertial Estimated (NED) [x, y]: [" + str(pos_center_inertial[0]) + "," + str(pos_center_inertial[1]) + "]"
        cv.putText(img, text, (0, 170), self.font_, 1, (255,0,255), 1)

        text = "Center Inertial REAL (NED) [x, y]: [" + str(self.real_center_[0]) + "," + str(self.real_center_[1]) + "]"
        cv.putText(img, text, (0, 190), self.font_, 1, (255,0,255), 1)

        text = "Yaw(Vehicle)"+ str(self.yaw_)
        cv.putText(img, text, (0, 210), self.font_, 1, (255,0,255), 1)

        #Compute Desired Yaw to Check
        pos_center_pixels = np.array([xc, yc])
        yaw_desired, atan_conta = self.computeDesiredYaw(pos_center_pixels, sonar_pos, self.yaw_)
        
        text = "Yaw Desired: "+ str(yaw_desired)
        cv.putText(img, text, (0, 230), self.font_, 1, (255,0,255), 1)

        text = "ATAN: " + str(atan_conta)
        cv.putText(img, text, (0, 250), self.font_, 1, (255,0,255), 1)

        cv.line(img, (int(sonar_pos[0]), int(sonar_pos[1])), (int(xc), int(yc)), (0, 255, 0), 2)

        text = "Distance Net: " + str(distance_net)
        cv.putText(img, text, (0, 270), self.font_, 1, (255,255,255), 1)

        text = "Dist Crit: " + str(self.dist_critical_)
        cv.putText(img, text, (0, 290), self.font_, 1, (255,255,255), 1)
        
        
        img_msg = self.bridge_.cv2_to_imgmsg(img, "bgr8")
        
        self.detection_image_pub_.publish(img_msg)
        

    def state_callback(self, data):
        self.surge_ = data.body_velocity.x
        self.sway_ = data.body_velocity.y
        self.yaw_ = data.orientation.z

        self.x_ = data.position.north - self.utm_pos_inertial_[0]
        self.y_ = data.position.east - self.utm_pos_inertial_[1]
        self.depth_ = data.position.depth

        msg = NED()
        msg.north = self.x_
        msg.east = self.y_
        msg.depth = self.depth_
        self.position_wrapped_pub_.publish(msg)

    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()

    def checkCenterOutlier(self, center):
        d = np.sqrt((self.last_center_[0]-center[0])**2 + (self.last_center_[1]-center[1])**2)
        if d < 2.5:
            return True
        else:
            self.counter_outlier_ = self.counter_outlier_ + 1
            return False

    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        try:
            # Compute Center of the Fishing Net in the Sonar Image
            try:
                detected_flag, xc, yc, img, distance_net, self.point_coordinates_, validity_center_flag, binary_img = \
                    self.sonar_detector.detect_circle(self.cv_image_, self.d_outlier_, self.nmin_points_, self.starting_point_)
            except:
                print("Exception: DETECT CIRCLE")

            self.detection_flag_pub_.publish(detected_flag)

            
            # Net Detected
            if detected_flag:
                self.estimation_counter_ = self.estimation_counter_ + 1
                height = img.shape[0]
                width = img.shape[1]

                # Compute Real Center in Body Frame
                pos_center_inertial, sonar_pos, center_pos_body = self.computeCenter(xc, yc, height, width)
                self.sonar_pos_ = sonar_pos

                not_outlier_flag = True
                # Check if the estimation of the center is not too far away from the previous one
                if self.last_center_ is not None:
                    not_outlier_flag = self.checkCenterOutlier(pos_center_inertial)

                # Show the computed circle
                self.pubImageWithCircle(xc, yc, pos_center_inertial, sonar_pos, center_pos_body, img, distance_net, binary_img, not_outlier_flag)
                
                #Center is Valid to Publish info
                if validity_center_flag and not_outlier_flag:
                    info_msg = DetectionResults()
                    info_msg.center_pixels = [xc, yc]
                    info_msg.center_inertial = pos_center_inertial
                    info_msg.sonar_pos_pixels = sonar_pos
                    info_msg.distance_net = distance_net

                    #Publish Info for the Inspection Controller
                    self.detection_info_pub_.publish(info_msg)

                    #update the initial point for Least Squares
                    self.starting_point_ = np.array([xc, yc])

                    #update last center
                    self.last_center_ = pos_center_inertial
            
            else:
                img_msg = self.bridge_.cv2_to_imgmsg(self.cv_image_, "bgr8")
                self.detection_image_pub_.publish(img_msg)

            print("Measure Counter | Outlier:" + str(self.estimation_counter_) + "|" + str(self.counter_outlier_))
            
        except:
            print("EXCEPTION: All needed values are not available")
            
        
def main():

    sonar_based_localisation = SonarBasedLocalisationNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
