#!/usr/bin/env python

import numpy as np

class InspectionController():
    def __init__(self, sonar_range, net_radius, k1, k2, dt) -> None:
        self.net_radius_ = net_radius
        self.sonar_range_ = sonar_range

        self.k1_ = k1
        self.k2_ = k2

        self.dt_ = dt
        
    
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

    def computeDesiredSurge(self, desired_distance, distance, last_distance, kp_dist, ki_dist):
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
        surge_desired = -kp_dist*error_dist - ki_dist * self.dt_ * (error_dist - last_error)
        # Saturate output
        if surge_desired > 0.7:
            surge_desired = 0.7
        elif surge_desired < -0.7:
            surge_desired = -0.7

        return surge_desired, error_dist


    """
        Uses Bump Function for smoothness -> to be infinite times derivative (C_infinity)
    """
    def swayReference(self, yaw, yaw_desired, error_dist, sway_desired):
        yaw_error0 = yaw_desired - yaw
        yaw_error = self.wrappYawError(yaw_error0)
        e_total = self.k1_*error_dist**2 + self.k2_ * yaw_error**2
        if e_total > 1:
            sway = 0.0
            print("Sway: 0")
        else:
            bump = np.exp(-1/(1-e_total**2))
            sway = sway_desired * np.exp(1) * bump
            print("Sway: " + str(sway))
        
        return sway, e_total
    
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

    

