#!/usr/bin/env python

import rospy
from sonar_based_localisation.msg import DetectionResults, FloatArray
from sonar_based_localisation_algorithms.InspectionController import InspectionController
from auv_msgs.msg import NavigationStatus, BodyForceRequest
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Wrench, Pose, Twist
from gazebo_msgs.msg import ModelStates
from farol_msgs.msg import mPidDebug
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

        self.inspection_controller_ = InspectionController(self.sonar_range_, self.net_radius_, self.k1_, self.k2_, self.dt_, self.real_center_, self.desired_distance_, self.sway_desired_)

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

        #Net Parameters
        self.real_center_ = rospy.get_param('~pos_center')
        self.net_radius_ = rospy.get_param('~net_radius')
        
        self.sonar_range_ = rospy.get_param('~sonar_range')
        
        self.sway_desired_ = rospy.get_param('~sway_desired')
        self.current_vel_ = rospy.get_param('~current_velocity')
        self.initial_depth_ = rospy.get_param('~initial_depth')
        self.heave_velocity_ = rospy.get_param('~heave_velocity')
        self.full_inspection_ = rospy.get_param('~full_inspection')
        
        #Distance
        self.desired_distance_ = rospy.get_param('~desired_distance')
        self.last_distance_net_ = None
        self.distance_array_ = None
        
        # Constants of the Function to choose reference for sway
        self.k1_ = rospy.get_param('~k1')
        self.k2_ = rospy.get_param('~k2')

        # Distance Controller
        self.kp_dist_ = rospy.get_param('~kp_dist')
        self.ki_dist_ = rospy.get_param('~ki_dist')
        self.kd_dist_ = rospy.get_param('~kd_dist')

        self.utm_pos_inertial_ = rospy.get_param('~utm_pos_inertial')
        self.vehicle_ = rospy.get_param('~Vehicle')
        self.inspection_flag_ = rospy.get_param('~inspection_flag')
        
        self.approach_flag_ = True # Flag to announce  the approaching phase
        self.stop_ = False # Flag to stop the movement of the vehicle
        self.descending_flag_ = False # Flag to stop the descending movement of the vehicle

        # Yaw Ref Smooth Function
        self.yaw_desired_ = None
        self.cumulative_dt_ = 0

        self.last_yaw_desired_ = None
        self.last_yaw_pub_ = None
        self.yaw_desired_derivative_ = None
        self.actual_time_ = None

        self.last_time_measure_ = None
        self.last_time_pub_ = None

        #Real yaw
        self.last_yaw_ideal_ = None
        self.last_yaw_ideal_time_ = None
        self.sway_ref_ = None
        self.sonar_pos_body_ = rospy.get_param('~sonar_pos_body')

        self.last_distance_time_ = None
        self.last_error_ = None
        
        self.first_detection_ = False
    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for SonarBasedLocalisationNode')
        rospy.Subscriber('/detection/object/detection_results', DetectionResults, self.detection_info_callback, queue_size=1)
        
        rospy.Subscriber(self.vehicle_+'/nav/filter/state', NavigationStatus, self.state_callback, queue_size=1)
        #rospy.Subscriber(self.vehicle_+'/gazebo/state', NavigationStatus, self.state_callback, queue_size=1)

        rospy.Subscriber('inspection_flag', Bool, self.inspection_flag_callback, queue_size=1)
        rospy.Subscriber('/detection/distance_net', Float64, self.distance_callback, queue_size=1)
        #rospy.Subscriber('/debug/real_distance', Float64, self.distance_callback, queue_size=1)
        
        
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for SonarBasedLocalisationNode')

        self.yaw_pub_ = rospy.Publisher(self.vehicle_ + '/ref/yaw', Float64, queue_size=1)
        self.surge_pub_ = rospy.Publisher(self.vehicle_ + '/ref/surge', Float64, queue_size=1)
        self.sway_pub_ = rospy.Publisher(self.vehicle_ + '/ref/sway', Float64, queue_size=1)
        self.depth_pub_ = rospy.Publisher(self.vehicle_ + '/ref/depth', Float64, queue_size=1)
        self.heave_pub_ = rospy.Publisher(self.vehicle_ + '/ref/heave', Float64, queue_size=1)

        self.error_dist_pub_ = rospy.Publisher('debug/error_dist', Float64, queue_size=1)
        self.avg_dist_pub_ = rospy.Publisher('debug/avg_dist', Float64, queue_size=1)
        self.e_total_pub_ = rospy.Publisher('/debug/sway_law/e_total', Float64, queue_size=1)
        self.dist_eterm_pub_ = rospy.Publisher('/debug/sway_law/dist_eterm', Float64, queue_size=1)
        self.yaw_eterm_pub_ = rospy.Publisher('/debug/sway_law/yaw_eterm', Float64, queue_size=1)
        self.desired_dist_pub_ = rospy.Publisher('/debug/desired_dist', Float64, queue_size=1)
        
        self.current_body_pub_ = rospy.Publisher('/current_velocity_body', FloatArray, queue_size=1)
        self.yaw_desired_derivative_pub_ = rospy.Publisher('/yaw_desired_derivative' , Float64, queue_size=1)
        self.yaw_ref_error_pub_ = rospy.Publisher('/yaw_ref_error', Float64, queue_size=1)

        self.new_filter_state_pub_ = rospy.Publisher(self.vehicle_ + '/nav/filter/new_state', NavigationStatus, queue_size=1)

        self.debug_dt_pub_ = rospy.Publisher("/debug/dt_between_measures", Float64, queue_size=1)
        self.real_dist_pub_ = rospy.Publisher("/real_distance", Float64, queue_size=1)

        self.debug_distance_pub_ = rospy.Publisher("/debug/distance", mPidDebug, queue_size=1)
        self.yaw_ideal_pub_ = rospy.Publisher("/debug/yaw_ideal", Float64, queue_size=1)
        self.yaw_ideal_d_pub_ = rospy.Publisher("/debug/yaw_ideal_d", Float64, queue_size=1)
        

    def initializeServices(self):
        rospy.loginfo('Initializing Services for SonarBasedLocalisationNode')
        self.distance_param_srv_ = rospy.Service('/change_dist_param', ChangeDistanceParameters, self.changeDistParamService)
        self.change_inspection_gains_srv_ = rospy.Service('/change_inspection_gains', ChangeInspectionGains, self.changeInspectionsGainsService)
    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    """
        Function to wrap the error in the [0, 360] interval
    """
    def wrappYawError(self, yaw_error0):
        yaw_error_translated = yaw_error0 - np.sign(yaw_error0)*360

        if abs(yaw_error0) <= abs(yaw_error_translated):
            yaw_error = yaw_error0
        elif abs(yaw_error0) > abs(yaw_error_translated):
            yaw_error = yaw_error_translated
        
        return yaw_error
    
    """
        Function to smooth the yaw reference: approximates to an affine function
            - step_dt: duration of the step in the yaw reference
            - dt: the 

    """
    def yawSmoothFcn(self, step_dt, measure_flag):
        actual_time = rospy.get_time()

        dt = actual_time - self.last_time_pub_
        # if there was a measure right before
        if measure_flag:
            self.cumulative_dt_ = 0
        else:
            self.cumulative_dt_ += dt

        #print("Measure Flag: " + str(measure_flag) + "| cumulative dt: " + str(self.cumulative_dt_))
        
        yaw_error_last = self.wrappYawError(self.yaw_desired_ - self.last_yaw_desired_)
        m = (yaw_error_last) / step_dt
        yaw_ref = m * self.cumulative_dt_ + self.last_yaw_desired_
        yaw_ref = self.inspection_controller_.wrapYaw(yaw_ref, 0, 360)
        
        #print("Yaw desired: " + str(self.yaw_desired_) + "| Last Yaw Desired: "+ str(self.last_yaw_desired_) + "Yaw Smooth: " + str(yaw_ref))
        if self.last_yaw_desired_ == yaw_ref:
            self.yaw_desired_derivative_ = 0
            self.yaw_desired_derivative_pub_.publish(self.yaw_desired_derivative_)
            print("\t\t\t yaw_ref igual last yaw")
        else:
            yaw_error_wrap = self.wrappYawError(yaw_ref - self.last_yaw_pub_)
            self.yaw_desired_derivative_ = yaw_error_wrap / dt
            self.yaw_desired_derivative_pub_.publish(self.yaw_desired_derivative_)

        self.last_time_pub_ = actual_time      
        return yaw_ref
    

    def addDistance(self, distance):
        if self.distance_array_ is None:
            self.distance_array_ = np.array([distance])
        elif self.distance_array_.size == 50:
            #delete oldest value in idx = 0
            aux_array = np.array([self.distance_array_[1:]])
            self.distance_array_ = np.append(aux_array, distance)
        else:
            self.distance_array_ = np.append(self.distance_array_, distance)
    
    def avgDistance(self):
        return np.sum(self.distance_array_)/self.distance_array_.size


    def computeRotationMatrix(self, yaw):
        yaw_radians = np.deg2rad(yaw)
        R = np.array([[np.cos(yaw_radians), -np.sin(yaw_radians)], [np.sin(yaw_radians), np.cos(yaw_radians)]])
        return R


    def makeNewNavState(self, data):
        current_body = self.computeBodyCurrent()
        state_msg = NavigationStatus()
        state_msg = data
        state_msg.body_velocity.x -= current_body[0,0]
        state_msg.body_velocity.y -= current_body[1,0]
        self.new_filter_state_pub_.publish(state_msg)
    
    
    def computeBodyCurrent(self):
        R = self.computeRotationMatrix(self.yaw_)
        R_I2B = np.transpose(R)
        current_inertial = np.array([[self.current_vel_[0]], [self.current_vel_[1]]])
        current_body = np.dot(R_I2B, current_inertial)
        return current_body
    
    
    def approachPhase(self):
        if abs(self.distance_net_-self.desired_distance_) < 0.2 and self.surge_ < 0.01:
            self.approach_flag_ = False
    
    def descendingPhase(self):
        pass
    
    def computeRealDistance(self, yaw):
        R = self.computeRotationMatrix(yaw)
        pos_vehicle_inertial = np.array([[self.x_], [self.y_]])
        pos_sonar_body = np.array([[self.sonar_pos_body_[0]], [self.sonar_pos_body_[1]]])
        pos_sonar_inertial = np.dot(R, pos_sonar_body) + pos_vehicle_inertial
        dist_center = np.sqrt((pos_sonar_inertial[0] - self.real_center_[0])**2 + (pos_sonar_inertial[1] - self.real_center_[1])**2)
        d = dist_center - self.net_radius_
        d = d[0]
        return d


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
            if self.first_detection_ is False:
                self.sway_pub_.publish(0.0)
                self.surge_pub_.publish(0.0)
            
            # Give Depth so the vehicle is stable
            if self.approach_flag_ or (not self.full_inspection_):
                self.depth_pub_.publish(self.initial_depth_)
                if abs(self.depth_ - self.initial_depth_) < 0.1:
                    self.heave_pub_.publish(0.0)
            else:
                if self.depth_< 6.0:
                    self.heave_pub_.publish(self.heave_velocity_)
                else:
                    self.depth_pub_.publish(6.0)
                    print("Estável a 6 m de depth")
                    self.stop_ = True
            tnow = rospy.get_time()

            # Uncomment this for smoothing function
            if self.last_time_measure_ is not None:
                yaw_smooth = self.yawSmoothFcn(0.8, False)
                self.yaw_pub_.publish(yaw_smooth)
                self.last_yaw_pub_ = yaw_smooth
            
            # self.yaw_pub_.publish(self.yaw_desired_)
            # self.last_yaw_pub_ = self.yaw_desired_

            if self.last_distance_time_ is None:
                duration = 0
            else:
                duration = tnow - self.last_distance_time_

            # Desired Surge
            surge_desired, error_dist = self.inspection_controller_.computeDesiredSurge(self.desired_distance_, self.distance_net_, self.last_error_, duration, self.kp_dist_, self.ki_dist_, self.kd_dist_)
            self.surge_pub_.publish(surge_desired)
            self.debug_distance_pub_.publish(self.inspection_controller_.getDebugMsg())
        
            #Check if its in aproaching phase
            sway_ref, e_total, dist_eterm, yaw_eterm = self.inspection_controller_.swayReference(self.yaw_, self.last_yaw_pub_, error_dist, self.sway_desired_)
            
            self.approachPhase()
            if self.approach_flag_ or self.stop_:
                sway_ref = 0.0

            self.sway_pub_.publish(sway_ref)
            self.sway_ref_ = sway_ref

            # update last distance to net
            self.last_distance_net_ = self.distance_net_

            # DEBUG TERMS
            self.e_total_pub_.publish(e_total)
            self.dist_eterm_pub_.publish(dist_eterm)
            self.yaw_eterm_pub_.publish(yaw_eterm)
            self.error_dist_pub_.publish(error_dist)
            self.avg_dist_pub_.publish(self.avg_distance_)
            self.desired_dist_pub_.publish(self.desired_distance_)

            self.last_distance_time_ = tnow
            self.last_distance_net_ = self.distance_net_
            self.last_error_ = error_dist
            
        except:
            print("EXCEPTION: Not all variables defined to compute desired yaw!")



    # Rate of receiving is about 1 second
    def detection_info_callback(self, data):
        self.first_detection_ = True

        #Circle Detection Data
        self.center_pixels_ = data.center_pixels
        self.center_inertial_ = data.center_inertial
        self.sonar_pos_pixels_ = data.sonar_pos_pixels

        # Save last yaw desired
        self.last_yaw_desired_ = self.yaw_desired_
        
        #Publish when a NEW measurement is done
        self.yaw_desired_ = self.inspection_controller_.computeDesiredYaw(self.center_pixels_, self.sonar_pos_pixels_, self.yaw_)

        
        # Check outliers
        if self.last_yaw_desired_ is not None:
            self.actual_time_ = rospy.get_time()            
            dt = self.actual_time_ - self.last_time_measure_
            
            txt = "Actual time: {actual_time: .5f} | Last Time: {last_time: .5f} | dt: {dt1: .5f}".format(actual_time = self.actual_time_\
                                                ,last_time=self.last_time_measure_, dt1=dt)
            print(txt)
            
            # Correcting Delay of sonar
            if self.sway_ > 0.5 * self.sway_desired_:
                yaw_rate_nominal = self.inspection_controller_.computeNominalYawRate()
                correction_term = yaw_rate_nominal * dt
                self.yaw_desired_ += correction_term

            # Computing Error between actual and last reference
            yaw_error = self.yaw_desired_ - self.last_yaw_desired_
            yaw_error_wrap = self.wrappYawError(yaw_error)
            
            # DeadReckoning
            arc_angle = (dt * self.sway_ / (self.distance_net_ + self.net_radius_)) * 180 / np.pi
            yaw_desired_dr = self.last_yaw_desired_ - arc_angle


            # Check Outlier

            # If vehicle is not moving sideways -> criteria of outliers tickens
            if self.sway_ < 0.2*self.sway_desired_:
                if (yaw_error_wrap < -arc_angle) or (yaw_error_wrap > arc_angle):
                    # Update with Deadreckoning
                    # Approximate the path to a arc of circunference
                    print("Outlier YAW! Error: " + str(yaw_error_wrap) + "-> Using Deadreckoning")
                    
                    print("\t\tYaw Desired do Deadreckoning: " + str(yaw_desired_dr)+ "| Yaw Medido: " + str(self.yaw_desired_))
                    self.yaw_desired_ = yaw_desired_dr
                    self.yaw_desired_ = self.inspection_controller_.wrapYaw(self.yaw_desired_, 0, 360)
                    # New error
                    yaw_error = self.yaw_desired_ - self.last_yaw_desired_
                    yaw_error_wrap = self.wrappYawError(yaw_error)
            else:
                if (yaw_error_wrap < -arc_angle - 2) or (yaw_error_wrap > arc_angle + 2):
                    # Update with Deadreckoning
                    # Approximate the path to a arc of circunference
                    print("2Outlier YAW! Error: " + str(yaw_error_wrap) + "-> Using Deadreckoning")
                    
                    print("\t\tYaw Desired do Deadreckoning: " + str(yaw_desired_dr)+ "| Yaw Medido: " + str(self.yaw_desired_))
                    self.yaw_desired_ = yaw_desired_dr
                    self.yaw_desired_ = self.inspection_controller_.wrapYaw(self.yaw_desired_, 0, 360)
                    # New error
                    yaw_error = self.yaw_desired_ - self.last_yaw_desired_
                    yaw_error_wrap = self.wrappYawError(yaw_error)


            # Now that I have the Desired Yaw, Time to smooth the function
            # Warning: I specified the time step duration, but its quite random ~ 1 second
            
            # Uncomment this for smooth function 
            yaw_smoothed = self.yawSmoothFcn(0.8, True)
            self.last_yaw_pub_ = yaw_smoothed
            self.yaw_pub_.publish(yaw_smoothed)

            # Uncomment this for without smooth function 
            # self.last_yaw_pub_ = self.yaw_desired_
            # self.yaw_pub_.publish(self.yaw_desired_)

            self.yaw_ref_error_pub_.publish(yaw_error_wrap)

            print("\t\tPublicado: " + str(self.yaw_desired_))
            
            self.last_time_measure_ = self.actual_time_

            #publish dt
            self.debug_dt_pub_.publish(dt)
            
        # First Measure
        else:
            self.last_time_measure_ = rospy.get_time()
            self.yaw_pub_.publish(self.yaw_desired_)
            self.last_yaw_pub_ = self.yaw_desired_
            self.last_time_pub_ = self.last_time_measure_
        
        
    def distance_callback(self, data):
        self.distance_net_ = data.data
        # Add Distance to the Distance Array
        self.addDistance(self.distance_net_)
        self.avg_distance_ = self.avgDistance()

    
    def state_callback(self, data):
        self.surge_ = data.body_velocity.x
        self.sway_ = data.body_velocity.y
        self.yaw_ = data.orientation.z

        self.x_ = data.position.north - self.utm_pos_inertial_[0]
        self.y_ = data.position.east - self.utm_pos_inertial_[1]
        self.depth_ = data.position.depth
        self.makeNewNavState(data)
        
        yaw_desired_real = self.inspection_controller_.computeRealDesiredYaw(self.x_, self.y_)
        self.yaw_ideal_pub_.publish(yaw_desired_real)

        #Compute Derivative
        if self.last_yaw_ideal_time_ is not None:
            tnow = rospy.get_time()
            yaw_d = (yaw_desired_real - self.last_yaw_ideal_) / (tnow - self.last_yaw_ideal_time_)
            self.last_yaw_ideal_time_ = tnow
            self.yaw_ideal_d_pub_.publish(yaw_d)
        else:
            self.last_yaw_ideal_time_ = rospy.get_time()
        
        self.last_yaw_ideal_ = yaw_desired_real
        


    def inspection_flag_callback(self, data):
        self.inspection_flag_ = data.data


    def changeDistParamService(self, request):
        if request.kp_dist < 0 or request.ki_dist < 0 or request.kd_dist < 0 or request.desired_distance <= 0:
            success = False
            message = "The values can't be negative"
        else:
            self.kp_dist_ = request.kp_dist
            self.ki_dist_ = request.ki_dist
            self.kd_dist_ = request.kd_dist
            self.desired_distance_ = request.desired_distance
            # reset integral term
            #self.inspection_controller_.resetDistIntegralTerm()
            
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
            self.inspection_controller_ = InspectionController(self.sonar_range_, self.net_radius_, self.k1_, self.k2_, self.dt_, self.real_center_, self.desired_distance_, self.sway_desired_)
            success = True
            message = "Inspection Gains changed successfuly"

        return success, message


def main():

    inspection_controller = InspectionControllerNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
