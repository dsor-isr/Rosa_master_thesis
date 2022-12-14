#!/usr/bin/env python

import cv2
import numpy as np
import os
import time
from numpy.linalg import norm
from detection_algorithms.tracker import *

class ObjectDetector:

    def __init__(self, config_path, weights_path, classes_path, score_threshold, size, principalPoint, focalDistance):

        """
        :param config_path:
        :param weights_path:
        :param classes_path:
        :param score_threshold:
        :param size:

        """

        self.config_path = config_path
        self.weights_path = weights_path
        self.classes_path = classes_path
        self.score_threshold = score_threshold
        self.image_size = size
        self.principalPoint = principalPoint
        self.focalDistance = focalDistance

        self.indexes = []
        
        self.optical_center = []
        
        # Create dicts to save information per frame
        self.frame_boxes = []
        self.frame_confidences = []
        self.frame_classes_ids = []
        self.frame_classes_names = []

        # set font of the text
        self.font = cv2.FONT_HERSHEY_PLAIN

        # get name(s) of the custom object(s)
        self.class_names = self._get_class()

        # Load Yolo
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)

        # Set CUDA as the preferable backend and target
        print("[INFO] setting preferable backend and target to CUDA...")
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # Create a tracker object
        self.tracker = EuclideanDistTracker()

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def draw_bboxes(self, frame, fps, Z):

        # set colors of the detection boxes
        colors = np.random.uniform(0, 255, size=(len(self.frame_boxes), 3))

        # Print the results (box, lable and confidence) on top of the image
        # ensure at least one detection exists
        if isinstance(self.indexes, np.ndarray):
            # loop over the indexes we are keeping
            for i in self.indexes.flatten():
                # extract the bounding box coordinates
                x, y, w, h = self.frame_boxes[i]
                # get lable of the object
                label = str(self.frame_classes_names[self.frame_classes_ids[i]])
                # get confidence of the detection
                confidence = str(round(self.frame_confidences[i], 2))
                
                # Get the id of the object
                ids = str(self.id[i])

                # get color
                color = colors[i]
                # draw a bounding box rectangle and lable on the frame
                cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
                cv2.putText(frame, label + " "+ids+ " " + confidence, (x, y+20), self.font, 1, (0, 0, 0), 2)
                # Display fps
                text_fps = "FPS: {:.2f}".format(fps)
                cv2.putText(frame, text_fps, (10, 30), self.font, 1, (0,0,0), 1)

                # Calculate the midle point of the object
                x_medium = int((x + x+w) / 2)
                y_medium = int((y + y+h) / 2)

                # Draw a + in the middle point of the object found
                cv2.line(frame , (x_medium, y_medium-1) , (x_medium, y_medium+1), (0, 255, 0), 1)
                cv2.line(frame , (x_medium-1, y_medium) , (x_medium+1, y_medium), (0, 255, 0), 1)

                # Draw a + on top of the optical center
                cv2.line(frame , (self.optical_center[0], self.optical_center[1]-1) , (self.optical_center[0], self.optical_center[1]+1), (0, 0, 255), 1)
                cv2.line(frame , (self.optical_center[0]-1, self.optical_center[1]) , (self.optical_center[0]+1, self.optical_center[1]), (0, 0, 255), 1)

                # Draw the object's translation vector
                cv2.line(frame , (x_medium, y_medium) , (self.optical_center[0], self.optical_center[1]), (255, 0, 0), 1)

                # Display the error in meters
                # Convert the midle point of the object in inertial coordinates relative to camera
                X_medium = ((x_medium - self.principalPoint[0])*Z) / self.focalDistance[0]
                Y_medium = ((y_medium - self.principalPoint[1])*Z) / self.focalDistance[1]
                # Calculate error in meters, NOTE: the optical_center in the inertial frame is the origin
                error_meters = norm(np.array([X_medium, Y_medium]))
                # Display the error
                text_error = "ERROR(meters): {:.2f}".format(error_meters)
                cv2.putText(frame, text_error, (10, 50), self.font, 1, (0,0,0), 1)

                # Display the depth
                text_error = "Depth(meters): {:.2f}".format(Z)
                cv2.putText(frame, text_error, (10, 70), self.font, 1, (0,0,0), 1)


        return frame

    def detect_object(self, frame):

        """

        :param image:
        :param visualize:
        :return:

        """

        # Reset all the dict that store iformation per image
        self.frame_boxes = []
        self.frame_confidences = []
        self.frame_classes_ids = []
        self.frame_classes_names = []
        self.id = []
        
        self.frame_boxes_nms = []
        self.frame_confidences_nms = []
        self.frame_classes_ids_nms = []
        self.frame_classes_names_nms = []

        # get the dimensions of the frame
        self.height, self.width, self.channels = frame.shape

        image_area = self.height * self.width

        # Calculate object dist to the optical center
        self.optical_center = [int(self.width/2), int(self.height/2)]

        #Prepare the image
        # other possible size, but worst in terms of detection is (128, 128)
        blob = cv2.dnn.blobFromImage(frame, 1/255,  self.image_size, (0,0,0), swapRB=True, crop=False)

        # Display 3 images with B, G, R intensities respectively
        for b in blob:
            for n, img_blob in enumerate(b):
                cv2.imshow(str(n), img_blob)

        # Pass blob as input into network
        self.net.setInput(blob)

        # get the output layers names
        layerOutputs = self.net.forward(self.output_layers)
        
        # Extrate the information from the layers Outputs
        for output in layerOutputs:
            # Extrate the infromation from the output
            for detection in output:
                # Every output contain 85 entries, the first 4 is about the location (the box) 
                # the fifth element is the box's confidence and the last 80 elements is the probability 
                # of the objet inside a box is one of the the 80 classes
                # Save the scores
                scores = detection[5:]
                # calculate the max score
                classes_id = np.argmax(scores)
                confidence = scores[classes_id]
                
                # Just trust in the object finded if the confidence is higher than 50%
                if confidence > self.score_threshold:
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    
                    # Find the upper-left corner of the box
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    # Update the list that save just the information of each frame
                    self.frame_boxes.append([x, y, w, h])
                    self.frame_confidences.append((float(confidence)))
                    self.frame_classes_ids.append(classes_id)
                    self.frame_classes_names.append(self.class_names[classes_id])

            # Find the redundent boxes: aplly non-maxima suppression to suppress weak, overlapping bounding box
            self.indexes = cv2.dnn.NMSBoxes(self.frame_boxes, self.frame_confidences, 0.5, 0.4)

            # Update the information of the detections after doing non-maxima suppression
            if isinstance(self.indexes, np.ndarray):
                # loop over the indexes we are keeping
                for i in self.indexes.flatten():
                    self.frame_boxes_nms.append(self.frame_boxes[i])
                    self.frame_confidences_nms.append(self.frame_confidences[i])
                    self.frame_classes_ids_nms.append(self.frame_classes_ids[i])
                    self.frame_classes_names_nms.append(self.frame_classes_names[i])


            # Get the id of the object // WERE TEST JUST FOR ONE OBJECT IN THE IMAGE
            if self.frame_boxes and self.frame_boxes_nms:
                boxes_ids = self.tracker.update(self.frame_boxes_nms)
                for box_id in boxes_ids:
                    _,_,_,_,id = box_id
                    self.id.append(id)

            
            return frame, self.frame_boxes_nms, self.frame_confidences_nms, self.frame_classes_names_nms, self.optical_center, self.id, image_area

       
