#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from detection_algorithms.ibvs import VisualServoing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from detection.msg import DetectionResults
import numpy as np
from std_msgs.msg import Float64, String, Int8, Bool
from uuv_sensor_ros_plugins_msgs.msg import DVL
from auv_msgs.msg import NavigationStatus
from gazebo_msgs.msg import ModelStates
import math

class IbvsNode():
    def __init__(self):
        """
        @.@ Init node
        """
        rospy.init_node('ibvs_node')

        self.loadParams()
        
        self.visual_servoing = VisualServoing(self.Kp, self.Ki, self.Kd, self.principalPoint, self.focalDistance, self.dt, self.rotation_b2c)
        
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)

        self.principalPoint = rospy.get_param('~principalPoint')
        self.focalDistance = rospy.get_param('~focalDistance')
        Kp = rospy.get_param('~Kp')
        Ki = rospy.get_param('~Ki')
        Kd = rospy.get_param('~Kd')
        self.Vel_Sat = rospy.get_param('~Velocity_Saturation')
        self.vehicle = rospy.get_param('~Vehicle')
        self.rotation_b2c = rospy.get_param('~Rotation_B2C')
        self.Kp = np.diag(Kp)
        self.Ki = np.diag(Ki)
        self.Kd = np.diag(Kd)
        self.dt = 1/self.node_frequency
        #for angular rate integration refs
        self.last_refs = np.array([0, 0, 0])
        # Starfish position in Gazebo
        self.starfish_position = np.array([0, 5, 5])
        # Body position in Gazebo
        self.position_body = np.array([0, 0, 0])
    

    """
    @.@ Member Helper function to set up subscribers;
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for DetectionNode')

        rospy.Subscriber('/detection/object/detection_result', DetectionResults, self.extract_corners_callback, queue_size=1)
        #subscribe vehicle state
        rospy.Subscriber(self.vehicle + '/bluerov_heavy0/nav/filter/state', NavigationStatus, self.state_callback, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback, queue_size=1)

    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for DetectionNode')

        self.surge_pub = rospy.Publisher(self.vehicle + '/ref/surge', Float64, queue_size=5)
        self.sway_pub = rospy.Publisher(self.vehicle + '/ref/sway', Float64, queue_size=5)
        self.heave_pub = rospy.Publisher(self.vehicle + '/ref/heave', Float64, queue_size=5)
        self.roll_pub = rospy.Publisher(self.vehicle + '/ref/roll', Float64, queue_size=5)
        self.pitch_pub = rospy.Publisher(self.vehicle + '/ref/pitch', Float64, queue_size=5)
        self.yaw_rate_pub = rospy.Publisher(self.vehicle + '/ref/yaw_rate', Float64, queue_size=5)


    def getVisualServoing(self):
        return self.visual_servoing
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)

    def initializeServices(self):
        pass

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

    def computeDesiredCorners(self, boxLocation):
        u_img = self.opticalCenter[0]
        v_img = self.opticalCenter[1]

        x = 200
        y = 200

        desired_corners = np.array([u_img-int(x/2), v_img-int(y/2), u_img+int(x/2), v_img-int(y/2), u_img+int(x/2), v_img+int(y/2)])
        print("Desired_corners: " + str(desired_corners) + "\n")
        return desired_corners
    
    def computeDetectedCorners(self, boxLocation):
        self.boxArea = boxLocation[2]*boxLocation[3]
        return np.array([boxLocation[0], boxLocation[1], boxLocation[0]+boxLocation[2], boxLocation[1], boxLocation[0]+boxLocation[2], boxLocation[1]+boxLocation[3]])

    def computeBodyVelocity(self, desired_corners, detected_corners, Z):
        desired_body_vel = self.visual_servoing.compute_body_vel(desired_corners, detected_corners, Z)
        surge = desired_body_vel[0]
        sway = desired_body_vel[1]
        heave = desired_body_vel[2]
        roll_rate = desired_body_vel[3]
        pitch_rate = desired_body_vel[4]
        yaw_rate = desired_body_vel[5]

        #integrar a la pata
        self.surge_pub.publish(surge)
        self.sway_pub.publish(sway)
        self.heave_pub.publish(heave)
        self.roll_pub.publish(self.last_refs[0] + self.dt*roll_rate)
        self.pitch_pub.publish(self.last_refs[1] + self.dt*pitch_rate)
        self.yaw_rate_pub.publish(yaw_rate)

    '''
    Function only to test with the real Z - in the end this Z should be estimated
    '''
    def computeZManually(self, position_star_fish, position_body):
        #C_Pq = R_I2C*(Pt_I - Pc_I)
        pass

    
    def gazebo_callback(self, data):
        aux = data.pose[3].position
        self.position_body = np.array([aux.y, aux.x, -aux.z])
        print(self.position_body)

    def state_callback(self, data):
        self.surge = data.body_velocity[0]
        self.sway = data.body_velocity[1]
        self.heave = data.body_velocity[2]
        self.roll = data.orientation[0]
        self.pitch = data.orientation[1]
        self.yaw = data.orientation[2]
        self.yaw_rate = data.orientation_rate[2]
        self.last_refs = [self.roll, self.pitch, self.yaw_rate]
        
        

            
    def extract_corners_callback(self, data):
        self.opticalCenter = data.opticalCenter
        self.imageArea = data.imageArea
        self.reset = data.lostObject
        location_box = data.location
        id_number = data.out_ids
        desired_corners = self.computeDesiredCorners(location_box)
        detected_corners = self.computeDetectedCorners(location_box) 
        self.computeBodyVelocity(desired_corners, detected_corners, 1.5)



def main():
    ibvs = IbvsNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
