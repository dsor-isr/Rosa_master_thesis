#!/usr/bin/env python

import numpy as np

class InspectionController():
    def __init__(self, sonar_range, net_radius, k1, k2, dt, real_center) -> None:
        self.net_radius_ = net_radius
        self.sonar_range_ = sonar_range
        self.real_center_ = real_center

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

    def computeDesiredSurge(self, desired_distance, distance, last_distance, kp_dist, ki_dist):
        
        # P Controller: u_desired = Kp*(d-d_desired) + ki * e * dt
        error_dist = -distance + desired_distance
        error_dist = self.sat(error_dist, -5.0, 2.0)
        self.integral_ += error_dist * self.dt_
        
        surge_desired = -kp_dist*error_dist - ki_dist * self.integral_
        Ka = 2.5
        # Saturate output
        if surge_desired > 0.3:
            self.integral_ += Ka*(0.3 - surge_desired) * self.dt_
            surge_desired = 0.3
        elif surge_desired < -0.3:
            self.integral_ += Ka*(-0.3 - surge_desired) * self.dt_
            surge_desired = -0.3

        return surge_desired, error_dist
    
    def computeDesiredThrustX(self, desired_distance, distance, last_distance, kp_dist, ki_dist):
        if last_distance is None:
            last_error = -distance + desired_distance
        else:
            last_error = -last_distance + desired_distance

        # P Controller: u_desired = Kp*(d-d_desired)
        error_dist = -distance + desired_distance
        
        # if the error is not far from the net do not give input of surge
        # This is to fix the "wavy" behaviour
        # if error_dist > -0.5 and error_dist < 0.3:
        #     surge_desired = 0
        # else:
        thrust_desired = -kp_dist*error_dist - ki_dist * self.dt_ * (error_dist - last_error)

        # Saturate output
        if thrust_desired > 3:
            thrust_desired = 3
        elif thrust_desired < -3:
            thrust_desired = -3

        return thrust_desired, error_dist


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
    

    def computeRealDesiredYaw(self, x, y, yaw):
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

    

