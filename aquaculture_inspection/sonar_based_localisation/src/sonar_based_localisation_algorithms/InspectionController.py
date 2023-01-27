#!/usr/bin/env python

import numpy as np

class InspectionController():
    def __init__(self, sonar_range, net_radius) -> None:
        self.net_radius_ = net_radius
        self.sonar_range_ = sonar_range
    

    
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
        return yaw_desired

    def computeDesiredSurge(self, desired_distance, distance):    
        # P Controller: u_desired = Kp*(d-d_desired)
        error_dist = distance - desired_distance
        surge_desired = 0.05*error_dist
        return surge_desired, error_dist

    '''
        Function to check if the yaw and the distance to the net is near the desired,
        so the inspection can begin
    '''
    def checkConditionsForSway(self, yaw, yaw_desired, error_dist):
        
        if np.abs(yaw_desired - yaw) < 5 and np.abs(error_dist) < 0.3:
            print("Conditions For Sway: Check")
            return True
        else:
            print("Conditions For Sway: Not fulfilled")
            return False
