#!/usr/bin/env python3

import numpy as np
import rospy
import math

class KalmanFilter(object):
    def __init__(self, init_x, init_y, std_vel, x_std_meas, y_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """

        # Intial State
        self.x = np.matrix([[init_x], [init_y], [0], [0]])

        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        #Initial Process Noise Covariance
        self.Q = np.matrix([[std_vel, 0, 1, 0],
                            [0, std_vel, 0, 1],
                            [1, 0, 0.1, 0],
                            [0, 1, 0, 0.1]])

        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2, 0],
                           [0, y_std_meas**2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.Q.shape[1])

        self.lastTime2 = rospy.get_rostime().to_sec()

    def predict(self, dt, u_x, u_y):
        
        # print("vx = {} \nvy = {}\n".format(round(u_x,2), round(u_y,2)))

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        
        # Define the Control Input Matrix B
        self.B = np.matrix([[-dt, 0],
                            [0, -dt],
                            [0, 0],
                            [0, 0]])

        # Define the  control input variables
        self.u = np.matrix([[u_x],[u_y]])

        # print("delta_pos : \n{}\n".format(np.dot(self.B, self.u)))
        # print("P_state: {}\n".format(self.x))
        # print("dt: {}\n".format(dt))

        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        
        return self.x

    def update(self, Zx, Zy):

        z = np.matrix([[Zx],
                       [Zy]])

        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))          
        
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))

        # print("erro : {}".format(z - np.dot(self.H, self.x)))

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P

        # print("U_state: {}\n".format(self.x))
        
        return self.x