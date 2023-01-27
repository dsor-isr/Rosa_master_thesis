#!/usr/bin/env python

import rospy
from sonar_based_localisation.msg import DetectionResults
from sonar_based_localisation_algorithms.InspectionController import InspectionController
from auv_msgs.msg import NavigationStatus
from std_msgs.msg import Float64, Bool
import numpy as np

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

        self.inspection_controller_ = InspectionController(self.sonar_range_, self.net_radius_)

        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeTimer()



    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        self.real_center_ = rospy.get_param('~pos_center')
        self.net_radius_ = rospy.get_param('~net_radius')
        self.desired_distance_ = rospy.get_param('~desired_distance')
        self.sonar_range_ = rospy.get_param('~sonar_range')
        self.sway_ref_ = rospy.get_param('~sway_ref')
        self.utm_pos_inertial_ = rospy.get_param('~utm_pos_inertial')
        self.vehicle_ = rospy.get_param('~Vehicle')
        self.inspection_flag_ = rospy.get_param('~inspection_flag')

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber('/detection/object/detection_results', DetectionResults, self.detection_info_callback, queue_size=1)
        rospy.Subscriber(self.vehicle_+'/nav/filter/state', NavigationStatus, self.state_callback, queue_size=1)
        rospy.Subscriber('inspection_flag', Bool, self.inspection_flag_callback, queue_size=1)
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for SonarBasedLocalisationNode')

        self.yaw_pub_ = rospy.Publisher(self.vehicle_ + '/ref/yaw', Float64, queue_size=1)
        self.surge_pub_ = rospy.Publisher(self.vehicle_ + '/ref/surge', Float64, queue_size=1)
        self.sway_pub_ = rospy.Publisher(self.vehicle_ + '/ref/sway', Float64, queue_size=1)
        self.error_dist_pub_ = rospy.Publisher('debug/error_dist', Float64, queue_size=1)

    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    def detection_info_callback(self, data):
        self.center_pixels_ = data.center_pixels
        self.center_inertial_ = data.center_inertial
        self.sonar_pos_pixels_ = data.sonar_pos_pixels
        self.distance_net_ = data.distance_net

    def state_callback(self, data):
        self.surge_ = data.body_velocity.x
        self.sway_ = data.body_velocity.y
        self.yaw_ = data.orientation.z

        self.x_ = data.position.north - self.utm_pos_inertial_[0]
        self.y_ = data.position.east - self.utm_pos_inertial_[1]
        self.depth_ = data.position.depth

    def inspection_flag_callback(self, data):
        self.inspection_flag_ = data.data

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
        try:
            yaw_desired = self.inspection_controller_.computeDesiredYaw(self.center_pixels_, self.sonar_pos_pixels_, self.yaw_)
            
            surge_desired, error_dist = self.inspection_controller_.computeDesiredSurge(self.desired_distance_, self.distance_net_)
            
            self.yaw_pub_.publish(yaw_desired)
            self.surge_pub_.publish(surge_desired)
            self.error_dist_pub_.publish(error_dist)
            
            if self.inspection_flag_:
                if self.inspection_controller_.checkConditionsForSway(self.yaw_, yaw_desired, error_dist):
                    # Conditions for Inspection Fulfilled
                    self.sway_pub_.publish(self.sway_ref_)
                else:
                    self.sway_pub_.publish(0.0)
            else:
                self.sway_pub_.publish(0.0)
                print("INspection Flag Not Activated")
            
        except:
            print("EXCEPTION: Not all variables defined to compute desired yaw!")


def main():

    inspection_controller = InspectionControllerNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()