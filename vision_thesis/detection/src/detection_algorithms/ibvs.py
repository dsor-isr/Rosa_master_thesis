#!/usr/bin/env python

import numpy as np
import numpy.matlib
from scipy.linalg import svdvals
import rospy
import time
from numpy.linalg import norm

class VisualServoing():

    def __init__(self, Kp, Ki, Kd, principalPoint, focalDistance, dt):
       #Inicialization
       self.L_matrix = np.matlib.zeros((6, 6)) #for 3 points
       
       #Visual Controller 
       self.Kp = Kp
       self.Ki = Ki
       self.Kd = Kd

       self.integralError = np.zeros((6,1))
       self.lastError = None
       self.derror_dt = np.zeros((6,1))
       self.error_meters = None
       self.mean_error_meters = np.array([0, 0])
       self.dt = dt

    
    def generate_L(self, desired_corners, Z):
        for i in range(0,3):
            u_img = desired_corners[2*i]
            v_img = desired_corners[2*i+1]
            x, y = self.convert_pixels_in_xy(u_img, v_img)
            self.L_matrix[2*i:2*i+1, :] = np.L_matrix([[-1/Z, 0, x/Z, x*y, -(1+x**2), y], \
                                                    [0, -1/Z, y/Z, 1+y**2, -x*y, -x]])
        
        return self.L_matrix

    def convert_pixels_in_xy(self, u_img, v_img):
        x = (u_img - principalPoint[0]) / self.focalDistance[0]
        y = (v_img - principalPoint[1]) / self.focalDistance[1]
        return x, y
    
    def get_error(self, desired_corners, detected_corners):
        # e = measured - ref
        error = detected_corners - desired_corners
        
        if (self.lastError is not None):
            # Compute integral Error with Trapezoid Rule
            self.integralError += (error + self.lastError)*dt/2
            # Compute derivative Error
            self.derror_dt = (error - self.lastError)/dt

        self.lastError = error

        return error
    
    def compute_desired_pixel_vel(self, desired_corners, detected_corners):
        desired_pixel_vel = np.zeros((6, 1))

        error = self.get_error(desired_corners, detected_corners)
        proportional = -np.dot(self.Kp, error)
        derivative = -np.dot(self.Kd, self.derror_dt)
        integral = -np.dot(self.Ki, self.integralError)

        desired_pixel_vel = proportional + derivative + integral
        return desired_pixel_vel

    def compute_new_vel(self, desired_corners, detected_corners, Z):
        desired_pixel_vel = self.compute_desired_pixel_vel(desired_corners, detected_corners)
        desired_camera_vel = np.zeros((6,1))

        L = self.generate_L(desired_corners, Z)

        desired_camera_vel = np.dot(np.linalg.inv(L), desired_camera_vel)
        return desired_camera_vel

