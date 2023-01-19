#!/usr/bin/env python

import numpy as np
import cv2 as cv
from scipy import optimize
import matplotlib.pyplot as plt
import sys

class sonarBasedNetCenterDetection:
    def __init__(self, sonar_range, net_radius) -> None:
        self.sonar_range = sonar_range
        self.net_radius = net_radius
        
    
    def grayFilter(self, frame):
        self.gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        return self.gray
    
    '''
        d: Diameter of each pixel neighborhood that is used during filtering.
        sigmaColor:	Filter sigma in the color space.
        sigmaSpace	Filter sigma in the coordinate space.
        URL: https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed
    '''
    def bilateralFilter(self, img, d, sigmaColor, sigmaSpace):
        self.bilateral_img = cv.bilateralFilter(img, d, sigmaColor, sigmaSpace)
        return self.bilateral_img
    
    '''
        URL: https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html#gae8a4a146d1ca78c626a53577199e9c57
        thresh: threshold value
        maxval: maxium value to use with THRESH_BINARY
    '''
    def binaryFilter(self, img, thresh, maxval):
        (T, self.binary_img) = cv.threshold(img, thresh, maxval, cv.THRESH_BINARY)
        return self.binary_img

    def plotImages(self):
        numpy_horizontal_concat = np.concatenate((self.gray, self.bilateral_img, self.binary_img), axis=1)
        cv.imshow('Original | Gray | Bilateral Filter | Binary Filter', numpy_horizontal_concat)
        cv.imshow('Original', self.img)
        cv.waitKey(0)
        cv.destroyAllWindows()

    def computeRadiusPxValue(self, img):
        height = img.shape[0]
        real_radius_pixels = height*self.net_radius/self.sonar_range
        return real_radius_pixels

    def computeVehiclePixelPos(self, width, height):
        sonar_pos = np.array([width/2.0, height])
        return sonar_pos
        

    def getWhitePointsBinaryImg(self, binary_img):
        #Indexes where the value is 255
        indexes = np.where(binary_img==255)
        x = np.array(indexes[0])
        y = np.array(indexes[1])
        
        x = np.reshape(x, (-1, 1))
        y = np.reshape(y, (-1, 1))
        
        point_coordinates = np.concatenate((y, x), axis=1)
        return point_coordinates

    def my_least_squares_circle(self, point_coordinates, radius):
        def fcn_optimize(c):
            return np.sqrt((point_coordinates[:, 0] - c[0])**2 + (point_coordinates[:, 1] - c[1])**2) - radius

        center_estimate = 200, 200
        center_opt, _ = optimize.leastsq(fcn_optimize, x0=center_estimate)
        a, b = center_opt

        return a, b

    def computeCircleSSE(self, point_coordinates, a, b, radius):
        n_data = point_coordinates.shape[0]
        sse = 0
        for i in range(0, n_data):
            sse = (radius - np.sqrt((point_coordinates[i, 0] - a)**2 + (point_coordinates[i, 1] - b)**2))**2
        
        print("Mean SSE: " + str(sse/n_data))
        return sse/n_data
    
    def filterInnerPoints(self, point_coordinates, a, b, radius, d_outlier):
        n_data = point_coordinates.shape[0]
        filtered_points = np.empty((0,2), int)
        for i in range(0, n_data):
            r_i = np.sqrt((point_coordinates[i, 0] - a)**2 + (point_coordinates[i, 1] - b)**2)
            #remove outliers
            if abs(r_i - radius) < d_outlier:
                #Outter Circle Points
                if r_i >= radius:
                    aux = point_coordinates[i, :]
                    aux = np.reshape(aux, (1, 2))
                    filtered_points = np.append(filtered_points, aux, axis=0)

        return filtered_points

    def circleRegression(self, point_coordinates, radius, threshold, d_outlier):
        a_1, b_1 = self.my_least_squares_circle(point_coordinates, radius)
        sse_mean = self.computeCircleSSE(point_coordinates, a_1, b_1, radius)
        if sse_mean > threshold:
            filtered_points = self.filterInnerPoints(point_coordinates, a_1, b_1, radius, d_outlier)
            a_2, b_2 = self.my_least_squares_circle(filtered_points, radius)
            return a_1, b_1, a_2, b_2
        else:
            return a_1, b_1

    
    def makeCircle(self, binary_img, sse_threshold, d_outlier):
        real_radius_px = self.computeRadiusPxValue(binary_img)
        point_coordinates = self.getWhitePointsBinaryImg(binary_img)
        result = self.circleRegression(point_coordinates, real_radius_px, sse_threshold, d_outlier)

        result_size = len(result)
        if result_size > 2:
            a_1 = result[0]
            b_1 = result[1]
            a_2 = result[2]
            b_2 = result[3]
        else:
            a_1 = result[0]
            b_1 = result[1]
            
        if result_size == 2:
            return a_1, b_1
        else:
            return a_2, b_2
            
    def detect_circle(self, frame, sse_threshold, d_outlier):
        # Flip because the given image is fliped
        frame = cv.flip(frame, 1)

        #Image Processing
        gray = self.grayFilter(frame)
        bilateral_img = self.bilateralFilter(gray, d=10, sigmaColor=100, sigmaSpace=100)
        binary_img = self.binaryFilter(bilateral_img, thresh=50, maxval=255)

        #Circle Regression
        xc, yc = self.makeCircle(binary_img, sse_threshold, d_outlier)
        return xc, yc, frame, binary_img

    
    
