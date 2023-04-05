#!/usr/bin/env python

import rospy
from sonar_based_localisation.msg import DetectionResults, FloatArray
from sonar_based_localisation_algorithms.InspectionController import InspectionController
from auv_msgs.msg import NavigationStatus, BodyForceRequest
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Wrench
import numpy as np
from sonar_based_localisation.srv import ChangeDistanceParameters, ChangeInspectionGains

class InspectionControllerNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('inspection_controller_node')

        
        self.loadParams()

        self.inspection_controller_ = InspectionController(self.sonar_range_, self.net_radius_, self.k1_, self.k2_, self.dt_)

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

        self.real_center_ = rospy.get_param('~pos_center')
        self.net_radius_ = rospy.get_param('~net_radius')
        self.desired_distance_ = rospy.get_param('~desired_distance')
        self.sonar_range_ = rospy.get_param('~sonar_range')
        self.sway_desired = rospy.get_param('~sway_desired')
        self.current_vel_ = rospy.get_param('~current_velocity')
        self.last_distance_net_ = None
        self.distance_array_ = None
        self.last_yaw_desired_ = None

        # Constants of the Function to choose reference for sway
        self.k1_ = rospy.get_param('~k1')
        self.k2_ = rospy.get_param('~k2')

        # Distance Controller
        self.kp_dist_ = rospy.get_param('~kp_dist')
        self.ki_dist_ = rospy.get_param('~ki_dist')

        self.utm_pos_inertial_ = rospy.get_param('~utm_pos_inertial')
        self.vehicle_ = rospy.get_param('~Vehicle')
        self.inspection_flag_ = rospy.get_param('~inspection_flag')
        self.approach_flag_ = True

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber('/detection/object/detection_results', DetectionResults, self.detection_info_callback, queue_size=1)
        rospy.Subscriber(self.vehicle_+'/nav/filter/state', NavigationStatus, self.state_callback, queue_size=1)
        rospy.Subscriber('inspection_flag', Bool, self.inspection_flag_callback, queue_size=1)
        rospy.Subscriber('/detection/distance_net', Float64, self.distance_callback, queue_size=1)
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for SonarBasedLocalisationNode')

        self.yaw_pub_ = rospy.Publisher(self.vehicle_ + '/ref/yaw', Float64, queue_size=1)
        self.surge_pub_ = rospy.Publisher(self.vehicle_ + '/ref/surge', Float64, queue_size=1)
        self.sway_pub_ = rospy.Publisher(self.vehicle_ + '/ref/sway', Float64, queue_size=1)
        self.error_dist_pub_ = rospy.Publisher('debug/error_dist', Float64, queue_size=1)
        self.avg_dist_pub_ = rospy.Publisher('debug/avg_dist', Float64, queue_size=1)
        self.e_total_pub_ = rospy.Publisher('/debug/sway_law/e_total', Float64, queue_size=1)
        self.dist_eterm_pub_ = rospy.Publisher('/debug/sway_law/dist_eterm', Float64, queue_size=1)
        self.yaw_eterm_pub_ = rospy.Publisher('/debug/sway_law/yaw_eterm', Float64, queue_size=1)
        self.desired_dist_pub_ = rospy.Publisher('/debug/desired_dist', Float64, queue_size=1)
        self.current_body_pub_ = rospy.Publisher('/current_velocity_body', FloatArray, queue_size=1)

        self.new_filter_state_pub_ = rospy.Publisher(self.vehicle_ + '/nav/filter/new_state', NavigationStatus, queue_size=1)
        

    def initializeServices(self):
        rospy.loginfo('Initializing Services for SonarBasedLocalisationNode')
        self.distance_param_srv_ = rospy.Service('/change_dist_param', ChangeDistanceParameters, self.changeDistParamService)
        self.change_inspection_gains_srv_ = rospy.Service('/change_inspection_gains', ChangeInspectionGains, self.changeInspectionsGainsService)
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    # Rate of receiving is about 1 second
    def detection_info_callback(self, data):
        self.center_pixels_ = data.center_pixels
        self.center_inertial_ = data.center_inertial
        self.sonar_pos_pixels_ = data.sonar_pos_pixels

        #Publish when a new measurement is done
        yaw_desired = self.inspection_controller_.computeDesiredYaw(self.center_pixels_, self.sonar_pos_pixels_, self.yaw_)
        # Check outliers
        if (self.last_yaw_desired_ is not None) and abs(yaw_desired - self.last_yaw_desired_) < 10:
            self.yaw_pub_.publish(yaw_desired)
            self.last_yaw_desired_ = yaw_desired
        else:
            self.yaw_pub_.publish(yaw_desired)
        
        
    def distance_callback(self, data):
        self.distance_net_ = data.data

        # Add Distance to the Distance Array
        self.addDistance(self.distance_net_)
        self.avg_distance_ = self.avgDistance()

    def addDistance(self, distance):
        if self.distance_array_ is None:
            self.distance_array_ = np.array([distance])
        elif self.distance_array_.size == 10:
            #delete oldest value in idx = 0
            aux_array = np.array([self.distance_array_[1:]])
            self.distance_array_ = np.append(aux_array, distance)
        else:
            self.distance_array_ = np.append(self.distance_array_, distance)
    
    def avgDistance(self):
        return np.sum(self.distance_array_)/self.distance_array_.size

    def state_callback(self, data):
        self.surge_ = data.body_velocity.x
        self.sway_ = data.body_velocity.y
        self.yaw_ = data.orientation.z

        self.x_ = data.position.north - self.utm_pos_inertial_[0]
        self.y_ = data.position.east - self.utm_pos_inertial_[1]
        self.depth_ = data.position.depth
        self.makeNewNavState(data)

    def makeNewNavState(self, data):
        current_body = self.computeBodyCurrent()
        state_msg = NavigationStatus()
        state_msg = data
        state_msg.body_velocity.x -= current_body[0,0]
        state_msg.body_velocity.y -= current_body[1,0]
        self.new_filter_state_pub_.publish(state_msg)
    
    
    def computeRotationMatrix(self):
        yaw_radians = np.deg2rad(self.yaw_)
        R = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians)], [np.sin(yaw_radians), np.cos(yaw_radians)]])
        return R
    
    def computeBodyCurrent(self):
        R = self.computeRotationMatrix()
        R_I2B = np.transpose(R)
        current_inertial = np.array([[self.current_vel_[0]], [self.current_vel_[1]]])
        current_body = np.dot(R_I2B, current_inertial)
        return current_body
    

    def inspection_flag_callback(self, data):
        self.inspection_flag_ = data.data

    def approachPhase(self):
        if abs(self.distance_net_-self.desired_distance_) < 0.1 and self.surge_ < 0.03:
            self.approach_flag_ = False


    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()


    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        try:
            # current_body = self.computeBodyCurrent()
            # current_msg = FloatArray()
            # current_msg.array = [current_body[0,0], current_body[1,0]]
            # self.current_body_pub_.publish(current_body)
            
            yaw_desired = self.inspection_controller_.computeDesiredYaw(self.center_pixels_, self.sonar_pos_pixels_, self.yaw_)

            if self.last_yaw_desired_ is not None:
                yaw_desired = self.last_yaw_desired_ - self.dt_ * self.sway_ / (self.distance_net_ + self.net_radius_)
                self.yaw_pub_.publish(yaw_desired)
            
            # Desired Surge
            surge_desired, error_dist = self.inspection_controller_.computeDesiredSurge(self.desired_distance_, self.distance_net_, self.last_distance_net_, self.kp_dist_, self.ki_dist_)
            #surge_desired -= current_body[0, 0]
            self.surge_pub_.publish(surge_desired)
            

            sway_ref, e_total, dist_eterm, yaw_eterm = self.inspection_controller_.swayReference(self.yaw_, yaw_desired, error_dist, self.sway_desired)
            #Check if its in aproaching phase
            self.approachPhase()
            if self.approach_flag_:
                sway_ref = 0.0
            
            #sway_ref -= current_body[1, 0]
            self.sway_pub_.publish(sway_ref)
            
            # update last distance to net
            self.last_distance_net_ = self.distance_net_

            # DEBUG TERMS
            self.e_total_pub_.publish(e_total)
            self.dist_eterm_pub_.publish(dist_eterm)
            self.yaw_eterm_pub_.publish(yaw_eterm)
            self.error_dist_pub_.publish(error_dist)
            self.avg_dist_pub_.publish(self.avg_distance_)
            self.desired_dist_pub_.publish(self.desired_distance_)
            
            
            
            
        except:
            print("EXCEPTION: Not all variables defined to compute desired yaw!")


    def changeDistParamService(self, request):
        if request.kp_dist < 0 or request.ki_dist < 0 or request.desired_distance <= 0:
            success = False
            message = "The values can't be negative"
        else:
            self.kp_dist_ = request.kp_dist
            self.ki_dist_ = request.ki_dist
            self.desired_distance_ = request.desired_distance
            success = True
            message = "Distance Parameters changed successfuly"
        
        return success, message
    
    def changeInspectionsGainsService(self, request):
        if request.ke_dist < 0 or request.ke_yaw < 0:
            success = False
            message = "The values can't be negative"
        else:
            self.k1_ = request.ke_dist
            self.k2_ = request.ke_yaw
            self.inspection_controller_ = InspectionController(self.sonar_range_, self.net_radius_, self.k1_, self.k2_, self.dt_)
            success = True
            message = "Inspection Gains changed successfuly"

        return success, message


def main():

    inspection_controller = InspectionControllerNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
