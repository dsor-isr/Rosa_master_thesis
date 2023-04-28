#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from sonar_based_localisation_algorithms.SonarBasedLocalisationAlgorithm import sonarBasedNetCenterDetection
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from auv_msgs.msg import NavigationStatus, NED
from sonar_based_localisation.msg import DetectionResults, FloatArray
from sonar_based_localisation.srv import ChangeNetRadius
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
                            self.dist_critical_, self.dist_between_posts_, self.number_posts_)
        
      
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()
        self.initializeTimer()


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)
        self.dt_ = 1/self.node_frequency

        self.utm_pos_inertial_ = rospy.get_param('~utm_pos_inertial')
        self.real_center_ = rospy.get_param('~pos_center')
        
        self.sonar_range_ = rospy.get_param('~sonar_range')
        self.sonar_pos_body_ = rospy.get_param('~sonar_pos_body')
        self.net_radius_ = rospy.get_param('~net_radius')
        self.distance_net_ = None
        self.dist_between_posts_ = rospy.get_param('~dist_between_posts')
        self.desired_distance_ = rospy.get_param('~desired_distance')
        self.number_posts_ = rospy.get_param('~number_posts')

        self.dist_critical_ = rospy.get_param('~dist_critical')
        self.sway_desired_ = rospy.get_param('~sway_desired')
        
        self.d_outlier_ = rospy.get_param('~distance_of_outliers')
        self.nmin_points_ = rospy.get_param('~nmin_points')
        self.bridge_ = CvBridge()
        self.starting_point_ = None

        self.vehicle_ = rospy.get_param('~Vehicle')

        # Distance Function Corrector Params
        self.a_ = rospy.get_param('~a')
        self.b_ = rospy.get_param('~b')
        self.c_ = rospy.get_param('~c')

        self.font_ = cv.FONT_HERSHEY_PLAIN

        self.last_center_ = None
        self.last_distance_net_ = None
        self.last_yaw_desired_ = None
        self.distance_dr_ = None
        self.counter_outlier_ = 0
        self.estimation_counter_ = 0
        self.new_measurement_ = False
        self.last_angle_ = None

        #Just for debug
        self.last_img_time_ = None
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber(self.vehicle_ + '/multibeam/sonar_image_topic', Image, self.image_update_callback, queue_size=5)
        
        #rospy.Subscriber(self.vehicle_+'/gazebo/state', NavigationStatus, self.state_callback, queue_size=1)
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
        self.distance_net_pub_ = rospy.Publisher('/detection/distance_net', Float64, queue_size=1)
        self.position_wrapped_pub_ = rospy.Publisher('/position_wrapped', NED, queue_size=1)

        self.old_distance_net_pub_ = rospy.Publisher('/detection/old_distance_net', Float64, queue_size=1)


        ### TOPICS FOR DEBUGGING
        self.real_distance_pub_ = rospy.Publisher('/debug/real_distance', Float64, queue_size=1)
        self.distance_with_outliers_pub_ = rospy.Publisher('/deberror_distug/distance_with_outliers', Float64, queue_size=1)
        self.error_real_dist_pub_ = rospy.Publisher('/debug/error_real_dist', Float64, queue_size=1)
        self.real_distance_px_pub_ = rospy.Publisher('/debug/pixel/real_distance_px', Float64, queue_size=1)
        self.distance_px_pub_ = rospy.Publisher('/debug/pixel/distance_px', Float64, queue_size=1)
        self.error_real_dist_px_pub_ = rospy.Publisher('/debug/pixel/error_real_dist_px', Float64, queue_size=1)
        self.new_center_pub_ = rospy.Publisher('/new_center', FloatArray, queue_size=1)

        self.angle_post_pub_ = rospy.Publisher('/debug/angle_post', Float64, queue_size=1)
        self.outlier_flag_pub_ = rospy.Publisher('/debug/outlier_flag', Float64, queue_size=1)
        
    def initializeServices(self):
        rospy.loginfo('Initializing Services for SonarBasedLocalisationNode')
        self.distance_param_srv_ = rospy.Service('/change_net_radius', ChangeNetRadius, self.changeNetRadiusSrv)
        
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)

    
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
        xc_body = (yp-yc) * self.sonar_range_/heigth + 0.25
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
    
    """
        Compute the distance to the net knowing:
            - Position of the Vehicle and Sonar
            - Position of the Center of the net
            - Radius of the net
    """

    def computeRealCenterImg(self, img_height):
        R_B2I = self.computeRotationMatrix()
        R_I2B = np.transpose(R_B2I)
        vehicle_pos = np.array([[self.x_], [self.y_]])
        real_center = np.array([[self.real_center_[0]], [self.real_center_[1]]])
        pos_center_body = np.dot(R_I2B, real_center - vehicle_pos)

        xc_real_px = pos_center_body[1] * img_height / self.sonar_range_ + self.sonar_pos_px_[0]
        yc_real_px = -pos_center_body[0] * img_height / self.sonar_range_ + self.sonar_pos_px_[1]
        return np.array([[xc_real_px], [yc_real_px]])
    
    def computeRealDistance(self):
        R = self.computeRotationMatrix()
        pos_vehicle_inertial = np.array([[self.x_], [self.y_]])
        pos_sonar_body = np.array([[self.sonar_pos_body_[0]], [self.sonar_pos_body_[1]]])
        pos_sonar_inertial = np.dot(R, pos_sonar_body) + pos_vehicle_inertial
        
        dist_center = np.sqrt((pos_sonar_inertial[0] - self.real_center_[0])**2 + (pos_sonar_inertial[1] - self.real_center_[1])**2)
        d = dist_center - self.net_radius_
        d = d[0]
        return d

    """
        Function to compute the New center, when the regression is done with the points of the net
        and not the outter posts. Since the used radius is different then the inner radius (net cage radius)
    """
    def computeNewCenter(self, xc, yc, height):
        ## compute the difference between the radius and the distance between the estimated center and the post
        radius_px = self.sonar_detector.convertMeter2Pixels(height, self.net_radius_)
        l = np.sqrt((xc - self.centroid_[0,0])**2 + (yc - self.centroid_[0,1])**2) - radius_px
        alpha = np.arctan((xc - self.centroid_[0,0]) / (yc - self.centroid_[0,1]))

        xc_new = xc + l*np.sin(alpha)
        yc_new = yc + l*np.cos(alpha)
        return xc_new, yc_new


    def computeNewDist(self, xc, yc, sonar_pos, height, net_radius):
        x_sonar = sonar_pos[0]
        y_sonar = sonar_pos[1]
        net_radius_px = self.sonar_detector.convertMeter2Pixels(height, net_radius)
        y = np.sqrt(net_radius_px**2 - (x_sonar - xc)**2) + yc
        dist = self.sonar_detector.convertPixels2Meters(height, y_sonar - y)
        return dist
        
    '''
        Display Computed Circle info in the Sonar image
    '''
    def pubImageWithCircle(self, xc, yc, pos_center_inertial, sonar_pos, center_pos_body, img, binary_img, not_outlier_flag):
        color = (255, 0, 0)
        height = img.shape[0]
        width = img.shape[1]

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
            # center of the circle
            cv.circle(img, (int(xc), int(yc)), 2, color, 2)
        else:
            cv.circle(img, (int(xc), int(yc)), int(radius_px), (255, 255, 255), 2)
            # center of the circle
            cv.circle(img, (int(xc), int(yc)), 2, (255, 255, 255), 2)
        
        
        centroid_post = self.sonar_detector.getCentroidPost()
        if centroid_post is not None:
            dist_between_post_px = self.sonar_detector.convertMeter2Pixels(height, self.dist_between_posts_)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px), (0,0,255), 2)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px-10), (0,0,255), 2)
            cv.circle(img, (int(centroid_post[0]), int(centroid_post[1])), int(dist_between_post_px+10), (0,0,255), 2)
        
        second_post = self.sonar_detector.getSecondCentroidPost()
        if second_post is not None:
            cv.circle(img, (int(second_post[0]), int(second_post[1])), int(40), (255,0,127), 2)


        ##### Plot Real Center
        real_center_px = self.computeRealCenterImg(height)
        real_dist_meter = self.computeRealDistance()
        cv.circle(img, (int(real_center_px[0]), int(real_center_px[1])), 2, (255, 255, 0), 2)
        cv.circle(img, (int(real_center_px[0]), int(real_center_px[1])), int(radius_px), (255, 255, 0), 2)
        
        #publish distances in pixels
        real_dist_estimated_px = np.sqrt((sonar_pos[0]-real_center_px[0])**2 + (sonar_pos[1]-real_center_px[1])**2)-radius_px
        distance_net_px = int(self.distance_net_ * height / self.sonar_range_)

        # Plot Line of distance in image
        cv.circle(img, (int(self.centroid_[0,0]), int(self.centroid_[0,1])), 4, (255, 51, 153), 2)
        cv.line(img, (int(sonar_pos[0]), int(sonar_pos[1])), (int(self.centroid_[0,0]), int(self.centroid_[0,1])), (0, 255, 0), 2)
        cv.line(img, (0, int(self.centroid_[0,1])), (width, int(self.centroid_[0,1])), (153, 255, 51), 2)
        cv.line(img, (0, int(sonar_pos[1]-real_dist_estimated_px)), (width, int(sonar_pos[1]-real_dist_estimated_px)), (255, 153, 51), 2)

        text = "Estimated Distance (meters | pixels) {meters:.2f} | {px:.2f}".format(meters = self.distance_net_, px = distance_net_px)
        cv.putText(img, text, (0, int(sonar_pos[1]-distance_net_px) + 20), self.font_, 1, (153, 255, 51), 1)
        # real_dist_estimated_px is an array of one element [[x]] and is converted to print
        a = np.asarray(real_dist_estimated_px) 
        text = "Real Distance (meters | pixels) {meters:.2f} | {px:.2f}".format(meters = real_dist_meter, px = a.item())
        cv.putText(img, text, (0, int(sonar_pos[1]-real_dist_estimated_px) - 10), self.font_, 1, (255, 153, 51), 1)


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

        text = "Dist Crit: " + str(self.dist_critical_)
        cv.putText(img, text, (0, 310), self.font_, 1, (255,255,255), 1)
        
        
        img_msg = self.bridge_.cv2_to_imgmsg(img, "bgr8")
        
        self.detection_image_pub_.publish(img_msg)
        

    def checkCenterOutlier(self, center, xc, yc):
        d = np.sqrt((self.last_center_[0]-center[0])**2 + (self.last_center_[1]-center[1])**2)
        pos_center_pixels = np.array([xc, yc])
        yaw_desired = self.computeDesiredYaw(pos_center_pixels, self.sonar_pos_px_, self.yaw_)
        if (d < 2.0):
            self.counter_outlier_ = 0 # reset in the number of outliers

            return True
        else:
            self.counter_outlier_ = self.counter_outlier_ + 1
            return False
    
    # TODO: melhorar isto com a velocidade anterior!
    def checkDistanceOutlier(self, yaw_ideal, angle):
        atual_time = rospy.get_time()
        self.distance_with_outliers_pub_.publish(self.distance_net_)
        outlier = 0
        if (self.last_distance_net_ is not None) and (self.last_angle_ is not None):
            if abs(self.sway_) > 0.8*self.sway_desired_:
                if abs(self.last_distance_net_  - self.distance_net_) > 0.15:
                    if (self.last_angle_ - angle) > 25:
                        print("\t\tNOT Distance outlier3")
                        outlier = 4
                    else:
                        dt = atual_time - self.last_measure_time_
                        out_dist = self.distance_net_
                        self.distance_net_ = self.last_distance_net_ - dt * self.surge_
                        self.last_measure_time_ = atual_time
                        print("\t\tDistance outlier3")
                        print("\t\tOutlier Dist: " + str(out_dist) + "| Corrected Dist"+ str(self.distance_net_))
                        outlier = 3
                        
            elif abs(self.last_distance_net_-self.distance_net_) > 0.5:
                dt = atual_time - self.last_measure_time_
                out_dist = self.distance_net_
                print("\t\tDistance outlier1")
                print("\t\tOUrtlier Dist: " + str(out_dist))
                self.distance_net_ = self.last_distance_net_ - dt * self.surge_
                self.last_measure_time_ = atual_time
                print("\t\tCorrected Dist"+ str(self.distance_net_))
                outlier = 1
                
            
            # # If the distance does not inccrease is an outlier
            elif abs(self.sway_) < 0.20*self.sway_desired_ and (self.last_distance_net_-self.distance_net_ < -0.1):
                dt = atual_time - self.last_measure_time_
                self.distance_net_ = self.last_distance_net_ - dt * self.surge_
                self.last_distance_net_ = self.distance_net_
                self.last_measure_time_ = atual_time
                print("Distance outlier2")
                outlier = 2
            
                    
            self.distance_dr_ = None

        self.outlier_flag_pub_.publish(outlier)
        self.last_measure_time_ = atual_time

    def computePostAngle(self):
        centroid = self.centroid_
        xc = centroid[0,0]
        yc = centroid[0,1]
        
        xs = self.sonar_pos_px_[0]
        ys = self.sonar_pos_px_[1]
        
        angle = np.arctan2(ys - yc, -(xs - xc))
        return angle
    
    def newDistCalc(self, angle, distance_net):
        correction_term = self.a_ * angle**2 + self.b_ * angle + self.c_
        new_dist = distance_net - correction_term
        return new_dist
    

    def computeRealDesiredYaw(self, x, y):
        x_c = self.real_center_[0]
        y_c = self.real_center_[1]

        atan_term = np.arctan2(y_c - y, (x_c - x))*180/np.pi
        
        yaw_desired = atan_term

        # wrapp angle to [0, 360] interval
        if yaw_desired > 360:
            yaw_desired = yaw_desired - 360
        elif yaw_desired < 0:
            yaw_desired = yaw_desired + 360

        return yaw_desired

    
    def handleDistance(self):
        angle = self.computePostAngle() * 180 / np.pi
        self.angle_post_pub_.publish(angle)
        if self.sway_ > 0.8*self.sway_desired_:
            new_dist = self.newDistCalc(angle, self.distance_net_)
            print("New Dist! " + str(new_dist))
        else:
            new_dist = self.distance_net_
            print("Old Dist! " + str(new_dist))
        
        old_dist = self.distance_net_
        self.distance_net_ = new_dist

        yaw_ideal = self.computeRealDesiredYaw(self.x_, self.y_)

        
        self.checkDistanceOutlier(yaw_ideal, angle)
        
        self.old_distance_net_pub_.publish(old_dist)
        self.distance_net_pub_.publish(self.distance_net_)

        # Compute Real Distance to the Net
        real_dist_meter = self.computeRealDistance()
        self.real_distance_pub_.publish(real_dist_meter)

        self.error_real_dist_pub_.publish(real_dist_meter - self.distance_net_)

        self.last_distance_net_ = self.distance_net_
        self.last_angle_ = angle

    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        try:
            # Compute Center of the Fishing Net in the Sonar Image
            try:
                detected_flag, xc, yc, img, self.distance_net_, self.point_coordinates_, validity_center_flag, binary_img, self.centroid_ = \
                    self.sonar_detector.detect_circle(self.cv_image_, self.d_outlier_, self.nmin_points_, self.starting_point_)
            except:
                print("Exception: DETECT CIRCLE")

            self.detection_flag_pub_.publish(detected_flag)

            self.handleDistance()
            

            # Net Detected
            if detected_flag:
                self.estimation_counter_ = self.estimation_counter_ + 1
                height = img.shape[0]
                width = img.shape[1]

                # Compute Real Center in Body Frame
                pos_center_inertial, sonar_pos, center_pos_body = self.computeCenter(xc, yc, height, width)
                self.sonar_pos_px_ = sonar_pos

                not_outlier_flag = True
                # Check if the estimation of the center is not too far away from the previous one
                if (self.last_center_ is not None) and (self.last_yaw_desired_ is not None):
                    not_outlier_flag = self.checkCenterOutlier(pos_center_inertial, xc, yc)
                    
                
                if self.counter_outlier_ > 30:
                    self.last_center_ = None
                    self.counter_outlier_ = 0
                
                #Center is Valid to Publish info
                #print("Validity Center: " + str(validity_center_flag) + "|nott outlier: " + str(not_outlier_flag) + "| new measurement: " + str(self.new_measurement_) )
                if validity_center_flag and not_outlier_flag and self.new_measurement_:

                    #### check estimated center
                    # TODO: Not working, check this
                    if self.distance_net_ < self.dist_critical_:
                        xc_new, yc_new = self.computeNewCenter(xc, yc, height)
                        xc = xc_new
                        yc = yc_new
                        
                        
                    new_center_msg = FloatArray()
                    new_center_inertial, _ , _ = self.computeCenter(xc, yc, height, width)
                    new_center_msg.array = new_center_inertial
                    self.new_center_pub_.publish(new_center_msg)
                    
                    
                    info_msg = DetectionResults()
                    info_msg.center_pixels = [xc, yc]
                    info_msg.center_inertial = pos_center_inertial
                    info_msg.sonar_pos_pixels = sonar_pos

                    #update the initial point for Least Squares
                    self.starting_point_ = np.array([xc, yc])

                    #update last center
                    self.last_center_ = pos_center_inertial
                    self.last_yaw_desired_, _ = self.computeDesiredYaw(np.array([xc, yc]), self.sonar_pos_px_, self.yaw_)
                    
                    self.detection_info_pub_.publish(info_msg)
                    self.new_measurement_ = False

                # Show the computed circle
                self.pubImageWithCircle(xc, yc, pos_center_inertial, sonar_pos, center_pos_body, img, binary_img, not_outlier_flag)
            
            
            else:
                # publish image without detection
                img2 = np.zeros_like(img)
        
                img2[:,:,0] = binary_img
                img2[:,:,1] = binary_img
                img2[:,:,2] = binary_img

                img = img2
                # Draw used points
                n_data = self.point_coordinates_.shape[0]
                for i in range(0, n_data):
                    cv.circle(img, (int(self.point_coordinates_[i, 0]), int(self.point_coordinates_[i, 1])), int(1), (0,255,0), 2)    
                img_msg = self.bridge_.cv2_to_imgmsg(img, "bgr8")
                self.detection_image_pub_.publish(img_msg)


            # FOR DEBUG
            real_center_px = self.computeRealCenterImg(height)
            
            radius_px = self.sonar_detector.convertMeter2Pixels(height, self.net_radius_)

            real_dist_estimated_px = np.sqrt((sonar_pos[0]-real_center_px[0])**2 + (sonar_pos[1]-real_center_px[1])**2)-radius_px
            distance_net_px = int(self.distance_net_ * height / self.sonar_range_)

            self.real_distance_px_pub_.publish(real_dist_estimated_px)
            self.distance_px_pub_.publish(distance_net_px)
            self.error_real_dist_px_pub_.publish(real_dist_estimated_px - distance_net_px)

            
        except:
            print("EXCEPTION: All needed values are not available")

    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()


    
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

    
    def image_update_callback(self, data):
        try:
            self.cv_image_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
            
            if self.last_img_time_ is None:
                self.last_img_time_  = rospy.get_time()
                print("First Image Received")
            else:
                tnow = rospy.get_time()
                dt = tnow - self.last_img_time_
                print("Img Received ---- dt: " + str(dt))
                self.last_img_time_ = tnow
        except CvBridgeError as e:
            raise e
        
        height = self.cv_image_.shape[0]
        width = self.cv_image_.shape[1]
        if self.starting_point_ is None:
            sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, height)
            self.starting_point_ = self.computeInitialPos(height, sonar_pos)
            self.sonar_pos_px_ = sonar_pos
        # sonar_pos = self.sonar_detector.computeVehiclePixelPos(width, height)
        # self.starting_point_ = self.computeInitialPos(height, sonar_pos)

        self.new_measurement_ = True
            

    def changeNetRadiusSrv(self, request):
        self.net_radius_ = request.net_radius
        self.sonar_detector = sonarBasedNetCenterDetection(self.sonar_range_, self.net_radius_, \
                            self.dist_critical_, self.dist_between_posts_)
        success = True
        message = "Net Radius Changed Successfully"
        return success, message
        
def main():

    sonar_based_localisation = SonarBasedLocalisationNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
