#!/usr/bin/env python

import numpy as np
import cv2 as cv
from scipy import optimize
import matplotlib.pyplot as plt
import sys

class sonarBasedNetCenterDetection:
    def __init__(self, sonar_range, net_radius) -> None:
        self.sonar_range_ = sonar_range
        self.net_radius_ = net_radius
        
    
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

    def computeRadiusPxValue(self, height):
        real_radius_pixels = height*self.net_radius_/self.sonar_range_
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

    def chooseStartingPoint(self, point_coordinates, img_width):
        #compute sse from 2 corner points
        x0 = np.array([[0, 0], [img_width, 0]])
        sse1, sse2 = 0, 0
        for point in point_coordinates:
            d1 = np.sqrt((point[0]-x0[0,0])**2 + (point[1]-x0[0,1])**2)
            d2 = np.sqrt((point[0]-x0[1,0])**2 + (point[1]-x0[1,1])**2)
            sse1 = sse1 + d1
            sse2 = sse2 + d2
        
        if sse2 < sse1:
            return x0[1,:]
        else:
            return x0[0,:]

    def my_least_squares_circle(self, point_coordinates, radius, starting_point):
        def fcn_optimize(c):
            return np.sqrt((point_coordinates[:, 0] - c[0])**2 + (point_coordinates[:, 1] - c[1])**2) - radius

        center_estimate = starting_point[0], starting_point[1]
        center_opt, _ = optimize.leastsq(fcn_optimize, x0=center_estimate)
        a, b = center_opt

        return a, b

    def computeCircleSSE(self, point_coordinates, a, b, radius):
        n_data = point_coordinates.shape[0]
        sse = 0
        for i in range(0, n_data):
            sse = (radius - np.sqrt((point_coordinates[i, 0] - a)**2 + (point_coordinates[i, 1] - b)**2))**2
        
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

    def circleRegression(self, point_coordinates, radius, threshold, d_outlier, starting_point):
        a_1, b_1 = self.my_least_squares_circle(point_coordinates, radius, starting_point=starting_point)
        sse_mean = self.computeCircleSSE(point_coordinates, a_1, b_1, radius)
        if sse_mean > threshold:
            filtered_points = self.filterInnerPoints(point_coordinates, a_1, b_1, radius, d_outlier)
            a_2, b_2 = self.my_least_squares_circle(filtered_points, radius, starting_point=starting_point)
            return a_1, b_1, a_2, b_2
        else:
            return a_1, b_1

    
    def makeCircle(self, real_radius_px, point_coordinates,sse_threshold, d_outlier, nmin_points, starting_point):
        #Not enough points to do a Regression
        if point_coordinates.shape[0] < nmin_points:
            return None, None
        result = self.circleRegression(point_coordinates, real_radius_px, sse_threshold, d_outlier, starting_point)

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
            
    def detect_circle(self, frame, sse_threshold, d_outlier, nmin_points, starting_point):
        # Flip because the given image is fliped
        frame = cv.flip(frame, 1)

        #Image Processing
        gray = self.grayFilter(frame)
        bilateral_img = self.bilateralFilter(gray, d=10, sigmaColor=100, sigmaSpace=100)
        binary_img = self.binaryFilter(bilateral_img, thresh=50, maxval=255)

        
        img_height = binary_img.shape[0]
        img_width = binary_img.shape[1]
        real_radius_px = self.computeRadiusPxValue(img_height)
        point_coordinates = self.getWhitePointsBinaryImg(binary_img)

        #Compute the Distance to the Net
        distance_net = self.computeDistanceToNet(point_coordinates, img_width, img_height)
        #Circle Regression
        xc, yc = self.makeCircle(real_radius_px, point_coordinates, sse_threshold, d_outlier, nmin_points, starting_point)
        if xc is None and yc is None:
            detection_flag = False
            return detection_flag, xc, yc, frame, binary_img
        detection_flag = True
        return detection_flag, xc, yc, frame, distance_net


    def computeDistanceToNet(self, point_coordinates, img_width, img_height):
        # find the index of the nearest point in the frame
        indexes = np.where(point_coordinates[:, 1] == max(point_coordinates[:, 1]))
        # Indexes in form array([x1, x2, x3, x4, ...], )
        nearest_points = point_coordinates[indexes[0], :]
        
        # Compute Vehicle Pos in the IMage
        sonar_pos = self.computeVehiclePixelPos(img_width, img_height)
        distance_pixels = self.avgDistance(nearest_points, sonar_pos)
        
        #Convert Pixels To Meters
        distance_meters = distance_pixels * self.sonar_range_ / img_height
        
        return distance_meters
        
    def avgDistance(self, nearest_points, sonar_pos):
        dist = 0
        counter = 0
        for point in nearest_points:
            dist = dist + np.sqrt((point[0] - sonar_pos[0])**2 + (point[1] - sonar_pos[1])**2)
            counter = counter + 1
        avg_distance = dist / counter
        return avg_distance

        
