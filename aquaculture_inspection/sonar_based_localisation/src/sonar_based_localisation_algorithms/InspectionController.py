#!/usr/bin/env python

import numpy as np
from farol_msgs.msg import mPidDebug

class InspectionController():
    def __init__(self, sonar_range, net_radius, k1, k2, dt, real_center, desired_distance_, swayNominal) -> None:
        self.net_radius_ = net_radius
        self.sonar_range_ = sonar_range
        self.real_center_ = real_center
        self.desired_distance_ = desired_distance_
        self.swayNominal = swayNominal

        self.k1_ = k1
        self.k2_ = k2

        self.dt_ = dt
        self.integral_ = 0
        
    
    def computeDesiredYaw(self, center_pos_pixels, sonar_pos, yaw):
        x_c = center_pos_pixels[0]
        y_c = center_pos_pixels[1]
        x_sonar = sonar_pos[0]
        y_sonar = sonar_pos[1]
        

        atan_term = np.arctan2(y_sonar - y_c, -(x_sonar - x_c))*180/np.pi
        '''
            - yaw_rel < 0, if xc < xsonar
            - yaw_rel > 0, if xc > xsonar
        '''
        yaw_rel = 90 - atan_term
        # yaw_error = yaw_ref - yaw
        yaw_desired = yaw + yaw_rel

        # wrapp angle to [0, 360] interval
        if yaw_desired > 360:
            yaw_desired = yaw_desired - 360
        elif yaw_desired < 0:
            yaw_desired = yaw_desired + 360

        return yaw_desired
    
    def sat(self, error, min_e, max_e):
        if error > max_e:
            error = max_e
        elif error < min_e:
            error = min_e
        return error

    def computeDesiredSurge(self, desired_distance, distance, last_error, duration, kp_dist, ki_dist, kd_dist):
        # P Controller: u_desired = Kp*(d-d_desired) + ki * e * dt
        error_p = distance - desired_distance
        error_dist = self.sat(error_p, -5.0, 5.0)
        
        self.integral_ += error_dist * duration
        if last_error is not None:    
            derivative = (error_dist - last_error) / duration
        else:
            derivative = 0
        
        pTerm = kp_dist * error_dist
        iTerm = ki_dist * self.integral_
        dTerm = kd_dist * derivative

        surge_desired = pTerm + iTerm + dTerm
        Ka = 1/0.1
        # Saturate output
        if surge_desired > 0.3:
            print("-------AntiWindup")
            print("ANtes: " + str(self.integral_))
            self.integral_ += Ka*(0.3 - surge_desired) * duration
            print("Depois: " + str(self.integral_))
            surge_desired = 0.3
        elif surge_desired < -0.3:
            self.integral_ += Ka*(-0.3 - surge_desired) * duration
            surge_desired = -0.3


        # Debuging Message
        self.msg_debug_ = mPidDebug()
        self.msg_debug_.ref = desired_distance
        self.msg_debug_.ref_d = 0
        self.msg_debug_.error = error_p
        self.msg_debug_.error_saturated = error_dist
        self.msg_debug_.pTerm = pTerm
        self.msg_debug_.iTerm = iTerm
        self.msg_debug_.dTerm = dTerm
        self.msg_debug_.output = surge_desired

        return surge_desired, error_dist

    """
        Uses Bump Function for smoothness -> to be infinite times derivative (C_infinity)
    """
    def swayReference(self, yaw, yaw_desired, error_dist, sway_desired):
        yaw_error0 = yaw_desired - yaw
        yaw_error = self.wrappYawError(yaw_error0)
        dist_eterm = self.k1_*error_dist**2
        yaw_eterm = self.k2_ * yaw_error**2
        e_total = dist_eterm + yaw_eterm
        if e_total > 1:
            sway = 0.0
        else:
            bump = np.exp(-1/(1-e_total**2))
            sway = sway_desired * np.exp(1) * bump
            if sway_desired - sway <= 0.02:
                sway = sway_desired
        
        return sway, e_total, dist_eterm, yaw_eterm
    
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
    
    # Wrap Angle to the [min, max] interval
    def wrapYaw(self, yaw, min_val, max_val):
        if yaw > max_val:
            yaw -= 360
        elif yaw < min_val:
            yaw += 360
        return yaw
    

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
    

    def computeNominalYawRate(self):
        perimeter = 2 * np.pi * (self.net_radius_ + self.desired_distance_)

        delta_t = perimeter / self.swayNominal
        # Inverted sign
        # Anti Clock Wise is -1
        # Clock Wise is 1
        yaw_rate_nominal = -(360/ delta_t)
        return yaw_rate_nominal

    def resetDistIntegralTerm(self):
        self.integral_ = 0

    def getDebugMsg(self):
        return self.msg_debug_

    

