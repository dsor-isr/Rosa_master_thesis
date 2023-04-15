#!/usr/bin/env python

import numpy as np
import cv2 as cv
from scipy import optimize
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops
from scipy.spatial.distance import cdist

class sonarBasedNetCenterDetection:
    def __init__(self, sonar_range, net_radius, dist_critical, dist_between_posts, number_posts) -> None:
        self.sonar_range_ = sonar_range
        self.net_radius_ = net_radius
        self.dist_between_posts_ = dist_between_posts

        #Critical Distance to Chose the SSE Threshold Value
        self.dist_critical_ = dist_critical
        self.number_posts_ = number_posts
        self.centroid_post_ = None
        self.second_post_ = None
    
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

    def convertMeter2Pixels(self, height, measure):
        real_radius_pixels = height*measure/self.sonar_range_
        return real_radius_pixels

    def convertPixels2Meters(self, heigth, measure):
        measure_meters = measure * self.sonar_range_ / heigth
        return measure_meters

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

    """
        Function to Compute the Mean Squared Error of the Circle Regression
    """
    def computeCircleSSE(self, point_coordinates, a, b, radius):
        n_data = point_coordinates.shape[0]
        sse = 0
        for i in range(0, n_data):
            sse = (radius - np.sqrt((point_coordinates[i, 0] - a)**2 + (point_coordinates[i, 1] - b)**2))**2
        
        return sse/n_data
    
    """
        Function to choose the points *OUTSIDE* of the circle for the Circle Regression
    """
    def filterInnerPoints(self, point_coordinates, a, b, radius, d_outlier):
        n_data = point_coordinates.shape[0]
        filtered_points = np.empty((0,2), int)
        for i in range(0, n_data):
            r_i = np.sqrt((point_coordinates[i, 0] - a)**2 + (point_coordinates[i, 1] - b)**2)
            #remove outliers
            # if abs(r_i - radius) < d_outlier:
                #Outter Circle Points
            if r_i >= radius:
                aux = point_coordinates[i, :]
                aux = np.reshape(aux, (1, 2))
                filtered_points = np.append(filtered_points, aux, axis=0)

        return filtered_points

    """
        Compute the Circle that better fits the data points
    """
    def circleRegression(self, point_coordinates, radius, threshold, d_outlier, starting_point):
        a_1, b_1 = self.my_least_squares_circle(point_coordinates, radius, starting_point=starting_point)
        sse_mean = self.computeCircleSSE(point_coordinates, a_1, b_1, radius)
        
        # if the error is big, then it discard the inner points and do the regression again, until it has a low error
        counter_loops = 0 # number of corrections
        while sse_mean > threshold:
            if counter_loops == 4:
                break
            filtered_points = self.filterInnerPoints(point_coordinates, a_1, b_1, radius, d_outlier)
            a_1, b_1 = self.my_least_squares_circle(filtered_points, radius, starting_point=starting_point)
            sse_mean = self.computeCircleSSE(filtered_points, a_1, b_1, radius)
            counter_loops = counter_loops + 1
        return a_1, b_1

    
    def makeCircle(self, real_radius_px, point_coordinates, sse_threshold, d_outlier, nmin_points, starting_point):
        #Not enough points to do a Regression
        if point_coordinates.shape[0] < nmin_points:
            return None, None
        result = self.circleRegression(point_coordinates, real_radius_px, sse_threshold, d_outlier, starting_point)

        a_1 = result[0]
        b_1 = result[1]

        return a_1, b_1
        
            
    def detect_circle(self, frame, d_outlier, nmin_points, starting_point):
        # Flip because the given image is fliped
        frame = cv.flip(frame, 1)
        
        #Image Processing
        gray = self.grayFilter(frame)
        bilateral_img = self.bilateralFilter(gray, d=10, sigmaColor=100, sigmaSpace=100)
        binary_img = self.binaryFilter(bilateral_img, thresh=40, maxval=255)
        
        img_height = binary_img.shape[0]
        img_width = binary_img.shape[1]
        real_radius_px = self.convertMeter2Pixels(img_height, self.net_radius_)
        point_coordinates = self.getWhitePointsBinaryImg(binary_img)
        
        # Choose the points for the regression
        chosen_points, distance_net, centroid, new_dist = self.choosePoints(point_coordinates, binary_img)

        sse_threshold = 0.5
        #Circle Regression
        try:
            xc, yc = self.makeCircle(real_radius_px, chosen_points, sse_threshold, d_outlier, nmin_points, starting_point)
        except:
            print("Exception: Make Circle")
            exit()
        
        if xc is None or yc is None:
            detection_flag = False
            validity_center_flag = False
            return detection_flag, xc, yc, frame, distance_net, point_coordinates, validity_center_flag, binary_img, centroid, new_dist
        else:
            detection_flag = True
            validity_center_flag = self.checkValidityCenter(xc, yc, img_height, point_coordinates)
            return detection_flag, xc, yc, frame, distance_net, chosen_points, validity_center_flag, binary_img, centroid, new_dist


    def computeDistanceToNetPixel(self, blob_list, idx_min, sonar_pos):

        blob = blob_list[idx_min]

        centroid = np.array(blob.centroid)
        centroid = np.reshape(centroid, (1,2))
        
        # switch coordinates cause image coordinates are switched (x_img = y, y_img = x)
        aux = centroid[0,0]
        centroid[0,0] = centroid[0,1]
        centroid[0,1] = aux

        #distance_pixels = np.sqrt((centroid[0, 1] - sonar_pos[1])**2 + ((centroid[0, 0] - sonar_pos[0])**2))
        distance_pixels = np.sqrt((centroid[0, 1] - sonar_pos[1])**2)
        ### Centroid is for debuging
        return distance_pixels, centroid
    
    def computeDistanceToNetPixel3(self, blob_list, idx_min, sonar_pos):

        blob = blob_list[idx_min]

        blob = blob.coords
        blob_x = blob[:, 1]
        blob_y = blob[:, 0]
        blob = np.transpose([blob_x, blob_y])

        i_max = np.argmax(blob_y)
        distance_pixels = np.sqrt((blob_y[i_max]-15 - sonar_pos[1])**2)
        
        #just to run
        centroid = np.array([[blob_x[i_max], blob_y[i_max]-15]])
        
        ### Centroid is for debuging
        return distance_pixels, centroid
        
    def avgDistance(self, points, sonar_pos):
        dist = 0
        counter = 0
        for point in points:
            dist = dist + np.sqrt((point[0] - sonar_pos[0])**2 + (point[1] - sonar_pos[1])**2)
            counter = counter + 1
        avg_distance = dist / counter
        return avg_distance
    
    def computeDistanceToNetPixel2(self, blob_list, idx_min, sonar_pos):
        blob = blob_list[idx_min].coords
        blob_x = blob[:, 1]
        blob_y = blob[:, 0]
        blob = np.transpose([blob_x, blob_y])
        
        """
            sonar_pos is (2,) shaped array, but cdist needs to both arrays to have the same number of columns
        """
        sonar_pos = np.reshape(sonar_pos, (1, 2))
        dist_array = cdist(sonar_pos, blob)
        
        idx = np.argmin(dist_array)
        distance_pixels = dist_array[0, idx]
        return distance_pixels

    def avgHeight(self, points):
        height = 0
        counter = 0
        for point in points:
            height = height + point[1]
            counter = counter + 1
        avg_height = height / counter
        return avg_height

    def checkValidityCenter(self, xc, yc, img_height, points):
        # Not valid -> center behind the sonar
        if yc > img_height:
            print("\t\tBehind Sonar")
            return False
        else:
            avg_height = self.avgHeight(points)
            if avg_height < yc:
                return False
            else:
                return True

    def computeNewDist(self, centroid, sonar_pos):
        x_c = centroid[0, 0]
        y_c = centroid[0, 1]
        x_sonar = sonar_pos[0]
        y_sonar = sonar_pos[1]

        atan_term = np.arctan2(y_sonar - y_c, -(x_sonar - x_c))
        '''
            - yaw_rel < 0, if xc < xsonar
            - yaw_rel > 0, if xc > xsonar
        '''
        yaw_rel = np.pi/2 - atan_term
        theta = abs(yaw_rel)
        
        d_centroid = np.sqrt((y_sonar - y_c)**2 + (x_sonar - x_c)**2)
        d1 = np.sqrt((x_c - x_sonar)**2)

        alpha = np.pi/self.number_posts_
        h1 = np.tan(alpha) * d1
        h = np.cos(theta) * d_centroid
        
        new_dist = h - h1
        print("\t\t\tTheta: " + str(theta) + "| new_dist: " + str(new_dist))
        return new_dist
    
    

        
    """
        Function for chosing the adequated points for the regression
    
    """
    def choosePoints(self, point_coordinates, binary_img):
        dist_critical = self.dist_critical_
        labels = label(binary_img)
        region_props = regionprops(labels)
        
        img_height = binary_img.shape[0]
        img_width = binary_img.shape[1]
        sonar_pos = self.computeVehiclePixelPos(img_width, img_height)
    
        blob_list, list_centroids, list_distance, idx_min = self.computeNearestBlob(region_props, sonar_pos)
        post = self.checkFirstPost(idx_min, blob_list, list_centroids, list_distance)
        print("list centroids shape: " + str(list_centroids.shape))
        
        distance_net_px, centroid = self.computeDistanceToNetPixel(blob_list, idx_min, sonar_pos)
        distance = self.convertPixels2Meters(img_height, distance_net_px)
        distance_net_px = list_distance[idx_min]
        distance = self.convertPixels2Meters(img_height, distance_net_px)
        
        # New method to compute distance to the cylinder
        new_dist_px = self.computeNewDist(centroid, sonar_pos)
        new_dist = self.convertPixels2Meters(img_height, new_dist_px)
        
        
        # if post is not detected and the blob is a merge between post and the net
        if not post and (distance < dist_critical):
            distance_net_px, centroid = self.computeDistanceToNetPixel3(blob_list, idx_min, sonar_pos)
            distance = self.convertPixels2Meters(img_height, distance_net_px)
        
        
        if distance >  dist_critical:        
            points = point_coordinates
            self.centroid_post_ = None      
        else:
            try:
                points = self.filterPosts(img_height, blob_list, list_centroids, list_distance, idx_min)
            except:
                print("EXCEPTION: BLOBS")
        return points, distance, centroid, new_dist


    def computeNearestBlob(self, region_props, sonar_pos):
        list_centroids = np.empty((0,2), float)
        list_distance = np.empty((0,1), float)
        blob_list = []

        # compute the nearest blob to the sonar
        for x in region_props:
            centroid = np.array(x.centroid)
            centroid = np.reshape(centroid, (1,2))
            
            # switch coordinates cause image coordinates are switched (x_img = y, y_img = x)
            aux = centroid[0,0]
            centroid[0,0] = centroid[0,1]
            centroid[0,1] = aux
            
            # filter the small blobs
            if x.area > 5:
                list_centroids = np.append(list_centroids, centroid, axis=0)
                dist = np.sqrt((centroid[0, 0] - sonar_pos[0])**2 + (centroid[0, 1] - sonar_pos[1])**2)
                list_distance = np.append(list_distance, dist)
                blob_list.append(x)
        

        idx_min = np.argmin(list_distance)
        return blob_list, list_centroids, list_distance, idx_min
    
    def checkFirstPost(self, idx_min, blob_list, list_centroids, list_distance):
        post = False
        ### WARNING AUTOMATE THIS VALUE 300
        # Check if the area of the post is not too big
        if blob_list[idx_min].area < 700:
            for i in range(0, list_distance.size):
                if i == idx_min:
                    continue
                
                # a blob above of the closest
                if list_centroids[i, 1] < list_centroids[idx_min, 1]:
                    if abs(list_centroids[i, 0] - list_centroids[idx_min, 0]) < 70:
                        dist = np.sqrt((list_centroids[i, 0] - list_centroids[idx_min, 0])**2 + (list_centroids[i, 1] - list_centroids[idx_min, 1])**2)
                        if dist < 100:
                            print("\tÉ um Poste")
                            # For debug reasons/ to print in the image
                            self.centroid_post_ = list_centroids[idx_min, :]
                            post = True
                            break
        return post

    def checkSecondPost(self, idx_min, blob_list, list_centroids, list_distance, img_height):
        # check if there is another Post
        # Warning Not Working Well
        second_post = False
        idx_second = -1
        
        dist_between_post_px = self.convertMeter2Pixels(img_height, self.dist_between_posts_)
        """
            Height difference between 2 posts (in the image)
            height_diff = Radius - cos(theta)*Radius, where theta = 2*pi/number_of_posts
        """
        radius_px = self.convertMeter2Pixels(img_height, self.net_radius_)
        theta = 2*np.pi/self.number_posts_
        height_diff = radius_px - np.cos(theta)*radius_px
        for i in range(0, list_distance.size):
            if i == idx_min:
                continue
            dist = np.sqrt((list_centroids[i, 0] - list_centroids[idx_min, 0])**2 + (list_centroids[i, 1] - list_centroids[idx_min, 1])**2)
            # error between centroids of both posts must be within a certain threshold
            if abs(dist - dist_between_post_px) < 10:
                if abs(list_centroids[i, 1] - list_centroids[idx_min, 1]) < height_diff+15:
                    print("\t\tSECOND POST")
                    second_post = True
                    idx_second = i
                    self.second_post_ = list_centroids[i, :]
        return second_post, idx_second

    def getPointsFromBlobs(self, idx_min, post, idx_second, second_post, blob_list, list_distance):
        points = np.empty((0,2), int)
        # Get points from the blobs
        for i in range(0, list_distance.size):
            # skip this blob if it is a post
            if i == idx_min and post:
                continue
            elif i == idx_second and second_post:
                continue

            blob = blob_list[i].coords
            blob_x = blob[:, 1]
            blob_y = blob[:, 0]
            blob = np.transpose([blob_x, blob_y])

            points = np.append(points, blob, axis=0)
        return points

    def filterPosts(self, img_height, blob_list, list_centroids, list_distance, idx_min):

        self.second_post_ = None
        self.centroid_post_ = None
        
        # Bool - if a post is detected
        post = self.checkFirstPost(idx_min, blob_list, list_centroids, list_distance)

        second_post = False
        idx_second = -1
        if post:
            second_post, idx_second = self.checkSecondPost(idx_min, blob_list, list_centroids, list_distance, img_height)
        
        points = self.getPointsFromBlobs(idx_min, post, idx_second, second_post, blob_list, list_distance)

        return points

    def getCentroidPost(self):
        return self.centroid_post_

    def getSecondCentroidPost(self):
        return self.second_post_
