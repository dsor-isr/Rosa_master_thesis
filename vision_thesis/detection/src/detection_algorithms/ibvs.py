#!/usr/bin/env python

import numpy as np
import numpy.matlib
from scipy.linalg import svdvals
import rospy
import time
from numpy.linalg import norm

class VisualServoing(self, Kp, Ki, Kd, principalPoint, focalDistance):

    def __init__(self):
       #Inicialization
       self.L_matrix = np.matlib.zeros((6, 6)) #for 3 points
       
       #Visual Controller 
       self.Kp = Kp
       self.Ki = Ki
       self.Kd = Kd

       #self.cumulativeError = 