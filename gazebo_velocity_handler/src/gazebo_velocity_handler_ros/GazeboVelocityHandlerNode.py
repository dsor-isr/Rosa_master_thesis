#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_velocity_handler_algorithms.GazeboVelocityHandlerAlgorithm import my_generic_sum_function
from std_msgs.msg import Int8, Bool, Float64
from geometry_msgs.msg import Wrench, Pose, Twist
from gazebo_msgs.msg import ModelStates
from auv_msgs.msg import NavigationStatus

class GazeboVelocityHandlerNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('gazebo_velocity_handler_node')

        
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


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)
        self.vehicle_ = rospy.get_param('~Vehicle')
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for GazeboVelocityHandlerNode')

        #Test: Subscribe Gazebo velocities
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback, queue_size=1)


    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for GazeboVelocityHandlerNode')
        self.gazebo_vel_pub_ = rospy.Publisher(self.vehicle_ + "/gazebo/vel", Twist, queue_size=1)
        self.gazebo_yaw_pub_ = rospy.Publisher(self.vehicle_ + "/gazebo/yaw", Float64, queue_size=1)
        self.gazebo_state_pub_ = rospy.Publisher(self.vehicle_ + '/gazebo/state', NavigationStatus, queue_size=1)

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


    def gazebo_callback(self, data):
        twist = data.twist
        pose = data.pose
        
        if len(twist) > 2:
            bluerov_twist = twist[2]

            #Velocity
            vel_x_inertial = bluerov_twist.linear.x
            vel_y_inertial = bluerov_twist.linear.y
            vel_z_inertial = bluerov_twist.linear.z

            rate_roll_inertial = bluerov_twist.angular.x
            rate_pitch_inertial = bluerov_twist.angular.y
            rate_yaw_inertial = bluerov_twist.angular.z

            rate_roll_ned = rate_pitch_inertial * 180 / np.pi
            rate_pitch_ned = rate_roll_inertial * 180 / np.pi
            rate_yaw_ned = -rate_yaw_inertial * 180 / np.pi

            bluerov_orientation = pose[2].orientation
            # In quaternions
            orientation_list = [bluerov_orientation.x, bluerov_orientation.y, bluerov_orientation.z, bluerov_orientation.w]
            # Convert angles from quaternions to euler angles
            (roll_enu, pitch_enu, yaw_enu) = euler_from_quaternion (orientation_list)
            orientation_list_euler = [roll_enu, pitch_enu, yaw_enu]

            # Rotate velocities to NED and then to the body frame
            vel_x_ned, vel_y_ned, vel_z_ned, roll_ned, pitch_ned, yaw_ned = self.changeENU2NED(vel_x_inertial, vel_y_inertial, vel_z_inertial, roll_enu, pitch_enu, yaw_enu)
            
            surge, sway = self.inertial2Body(vel_x_ned, vel_y_ned, yaw_ned)
            yaw_ned = np.rad2deg(yaw_ned)
            

            # Position
            bluerov_pos = pose[2].position
            x_ned = bluerov_pos.y
            y_ned = bluerov_pos.x
            z_ned = -bluerov_pos.z

            msg_vel = Twist()
            msg_vel.linear.x = surge
            msg_vel.linear.y = sway
            msg_vel.linear.z = -vel_z_inertial

            self.gazebo_vel_pub_.publish(msg_vel)
            self.gazebo_yaw_pub_.publish(yaw_ned)

            state_msg = NavigationStatus()
            state_msg.body_velocity.x = surge
            state_msg.body_velocity.y = sway
            state_msg.body_velocity.z = vel_z_ned

            state_msg.orientation.x = self.wrapAngle(roll_ned * 180 / np.pi, 0, 360) 
            state_msg.orientation.y = self.wrapAngle(pitch_ned * 180 / np.pi, 0, 360) 
            state_msg.orientation.z = self.wrapAngle(yaw_ned, 0, 360) 

            state_msg.orientation_rate.x = rate_roll_ned
            state_msg.orientation_rate.y = rate_pitch_ned
            state_msg.orientation_rate.z = rate_yaw_ned

            state_msg.position.north = x_ned
            state_msg.position.east = y_ned
            state_msg.position.depth = z_ned

            self.gazebo_state_pub_.publish(state_msg)



    # Rotate Inertial frame from ENU to NED
    def changeENU2NED(self, vel_x, vel_y, vel_z, roll_enu, pitch_enu, yaw_enu):
        vel_x_ned = vel_y
        vel_y_ned = vel_x
        vel_z_ned = -vel_z

        roll_ned = pitch_enu
        pitch_ned = roll_enu
        yaw_ned = np.pi/2-yaw_enu
        return vel_x_ned, vel_y_ned, vel_z_ned, roll_ned, pitch_ned, yaw_ned

    # Rotate velocities from Inertial to Body frame
    def inertial2Body(self, vel_x, vel_y, yaw):
        surge = np.cos(yaw) * vel_x + np.sin(yaw) * vel_y
        sway = -np.sin(yaw) * vel_x + np.cos(yaw) * vel_y
        return surge, sway
    

    def wrapAngle(self, angle, min_value, max_value):
        if angle > max_value:
            angle -= 360
        elif angle < min_value:
            angle += 360
        
        return angle
    
    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        # REMOVE pass and do your magic
        pass
            


def main():

    gazebo_velocity_handler = GazeboVelocityHandlerNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
