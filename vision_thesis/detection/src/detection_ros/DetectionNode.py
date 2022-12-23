# #!/usr/bin/env python

# """ 
# Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
# """
# import rospy
# from detection.msg import DetectionResults
# from detection_algorithms.david_object_detector import ObjectDetector
# from std_msgs.msg import Int8, Bool, String, Float64
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from auv_msgs.msg import NavigationStatus
# from uuv_sensor_ros_plugins_msgs.msg import DVL
# import time
# import cv2

# # Defining how many time we want to stay hovering the object
# MAX_TIME_TO_GO_TO_OBJECT = 1500 #seg
# class DetectionNode():
#     def __init__(self):
#         """
#         Constructor for ros node
#         """
#         # Start the FPS Timer
#         self.starting_time = time.time()
#         # instanciate the counter of frame_ids
#         self.frame_ids = 0
#         """
#         @.@ Init node
#         """
#         rospy.init_node('detection_node')

        
#         """
#         @.@ Handy Variables
#         # Declare here some variables you might think usefull -> example: self.fiic = true
#         """
#         self.loadParams()
        
#         self.detector = ObjectDetector(self.config_path, self.weights_path, self.classes_path, self.score_threshold,\
#                 self.input_size, self.principalPoint, self.focalDistance)
#         """
#         @.@ Dirty work of declaring subscribers, publishers and load parameters 
#         """
        
#         self.initializeSubscribers()
#         self.initializePublishers()
#         self.initializeServices()


#     """
#     @.@ Member Helper function to set up parameters; 
#     """
#     def loadParams(self):
#         self.node_frequency = rospy.get_param('~node_frequency', 10)

#         self.bridge = CvBridge()

#         rospy.loginfo("Object Detection Initializing")
#         rospy.loginfo("Yolo V3")

#         self.visualize = rospy.get_param('~visualization')
#         self.config_path = rospy.get_param('~config_path')
#         self.weights_path = rospy.get_param('~weights_path')
#         self.classes_path = rospy.get_param('~classes_path')
#         self.score_threshold = rospy.get_param('~score_threshold')
#         self.input_size = (416, 416) #(512, 512)
#         self.principalPoint = rospy.get_param('~principalPoint')
#         self.focalDistance = rospy.get_param('~focalDistance')
        
#         self.Idle_flag = rospy.get_param('~Idle_flag')
#         self.Mission_flag = rospy.get_param('~Mission_flag')
#         self.Vision_flag = rospy.get_param('~Vision_flag')
#         self.WP_flag = rospy.get_param('~WP_flag')
#         self.vehicle = rospy.get_param('~Vehicle')


#         #COPIEI SEM VER
#         # set font of the text
#         self.font = cv2.FONT_HERSHEY_PLAIN
#         # Boolean that said if we lost the target or not, if yes we need to reset the cumulativeError
#         self.lostObject = True
#         # Instanciate the value for Flag mission
#         self.Flag = 0
#         # counter of number of frames we dont have a detection
#         self.framesWithoutObject = 0
#         # Creat a dictionary to the ID of the object and the state of the object (chasing, chased, lost, canceled)
#         self.objectsDict = {}
#         # Variable that indicate wich is the object we are chasing
#         self.idObj = None
#         # Instanciate the altitude value
#         self.altitude = 4.5
#         # Variable to store the last time
#         self.lastTime = rospy.get_rostime().to_sec()
#         self.dt = 0
#         self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT # segundos
#         self.oldYaw = None
#         self.lastMission = None
#         self.lastFlag = None
#         self.KalmanFilter = None
#         self.objVel = [0, 0]
#         self.img_croped = False
#         self.OnceFlag = 0
#         self.gamma_value = 0.0
        
    

#     """
#     @.@ Member Helper function to set up subscribers; 
#     """
#     def initializeSubscribers(self):
#         rospy.loginfo('Initializing Subscribers for DetectionNode')
#         rospy.Subscriber(self.vehicle+'/bluerov/camera/camera_image', Image, self.image_update_callback, queue_size=5)

#         rospy.Subscriber(self.vehicle+'/nav/filter/state', NavigationStatus, self.State_callback, queue_size=1)
#         # Subscribe the topic that store the mission
#         rospy.Subscriber(self.vehicle+'/addons/Mission_String', String, self.get_last_mission_callback, queue_size=1)

#         # Subscribe the topic Flag -> Tipo de Missao
#         rospy.Subscriber(self.vehicle+'/Flag', Int8, self.get_mission_Flag_callback, queue_size=1)
        

    
#     """
#     @.@ Member Helper function to set up publishers; 
#     """
#     def initializePublishers(self):
#         rospy.loginfo('Initializing Publishers for DetectionNode')
#         # Pub the detection
#         self.detection_image_pub = rospy.Publisher('/detection/object/detection_visualization/', Image, queue_size=1)
#         # Pub detection info
#         self.detection_results_pub = rospy.Publisher('/detection/object/detection_result', DetectionResults, queue_size=1)

#         # Pub Flag topic
#         self.Flag_pub = rospy.Publisher(self.vehicle+'/Flag', Int8, queue_size=1)

#     def initializeServices(self):
#             pass

#     """
#     @.@ Member helper function to set up the timer
#     """
#     def initializeTimer(self):
#         self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


#     """
#     @.@ Member helper function to shutdown timer;
#     """
#     def shutdownTimer(self):
#         self.timer.shutdown()


#     """
#     @.@ Timer iter callback. Where the magic should happen
#     """
#     def timerIterCallback(self, event=None):
#         # REMOVE pass and do your magic
#         pass

#     def image_update_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             raise e

#         #increment number of the frames
#         self.frame_ids += 1

#         #check if there is an image
#         if cv_image is None:
#             return

#         # run the detector
#         frame, out_boxes, out_scores, out_classes, self.optical_center, ids, self.imageArea = self.detector.detect_object(cv_image)

#         #check if an object was detected
#         if out_classes:
#             index = self.find_object_with_more_confidence(out_scores)

#             # get the centre of the object
#             self.index_box = out_boxes[index]
#             self.index_score = out_scores[index]
#             self.index_class = out_classes[index]
#             self.width = self.index_box[2]
#             self.height = self.index_box[3]
#             self.idObj = ids[index]

#             # check if the object is a new detected object
#             if len(self.objectsDict) == 0 or self.idObj not in self.objectsDict:

#                 # Add the obj into the dictionary
#                 self.objectsDict[self.idObj] = 'chasing'
            
#         #Plublishing the message with the detection to activate the IBVS Controller
#         self.detection_results_pub.publish(self.convert_results_to_message(self.predLoc, self.index_score, self.index_class,\
#                 self.optical_center, self.lostObject, 0, self.imageArea, self.objVel))
        
#         if self.visualize:
#             self.pub_frame(frame, True, out_boxes, out_scores, out_classes, ids)
    
    
#     def find_object_with_more_confidence(self, out_scores):
#         for i in range(len(out_scores)):
#             index = np.argmax(out_scores)
#         return index
    
#     def pub_frame(self, frame, existPredict, out_boxes, out_scores, out_classes, ids):
#         # Calculate FPS
#         elapsed_time = time.time() - self.starting_time
#         fps = self.frame_ids / elapsed_time
        
#         if out_classes:
#             self.draw_all_boxes(frame, out_boxes, out_scores, out_classes, ids)

#         if existPredict:
#             # draw the predicted box
#             cv2.rectangle(frame, (self.predLoc[0],self.predLoc[1]), 
#                     (self.predLoc[0]+self.width, self.predLoc[1]+self.height), (255,0,0), 2)

#             # display info of chosen box
#             self.display_info_of_chosen_box(frame, self.predLoc[0], self.predLoc[1],
#                     self.width, self.height)
           
#         if self.idObj is not None:
#             # Object State
#             text = "Obj: " + str(self.idObj) + " -> State: " + str(self.objectsDict[self.idObj])
#             cv2.putText(frame, text, (10, 110), self.font, 1, (0,0,0), 1)

#         # Display fps
#         text = "FPS: {:.2f}".format(fps)
#         cv2.putText(frame, text, (10, 30), self.font, 1, (0,0,0), 1)
#         # Display the depth
#         text = "Depth(meters): {:.2f}".format(self.altitude)
#         cv2.putText(frame, text, (10, 70), self.font, 1, (0,0,0), 1)
#         # Flag information
#         text = "Flag: {}".format(self.Flag)
#         cv2.putText(frame, text, (10, 90),self.font, 1, (0,0,0), 1)
#         # Time to Inspect
#         text = "Time left: " + str(self.timeToGoToObject)
#         cv2.putText(frame, text, (10, 130), self.font, 1, (0,0,0), 1)
#         # Yaw information
#         text = "RefYaw: " + str(self.oldYaw) + " | Yaw: " + str(round(self.yaw,2)) 
#         cv2.putText(frame, text, (10, 150), self.font, 1, (0,0,0), 1)
#         # Body Speed information
#         text = "Body Vel [x,y]: [" + str(round(self.bodySurge,2)) + "," + str(round(self.bodySway,2)) + "]"
#         cv2.putText(frame, text, (10, 170), self.font, 1, (0,0,0), 1)
#         # Obj Speed information
#         text = "OBJ Vel [x,y]: [" + str(round(self.objVel[0],2)) + "," + str(round(self.objVel[1],2)) + "]"
#         cv2.putText(frame, text, (10, 190), self.font, 1, (0,0,0), 1)
#         # Frames without detection
#         text = "Frames without detection: " + str(self.framesWithoutObject)
#         cv2.putText(frame, text, (10, 210), self.font, 1, (0,0,0), 1)
#         # Convert the frame to a img message/home/dgd_rosa/.local/lib/python3.8/site-packages
#         img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
#         #Publish the frame
#         self.detection_image_pub.publish(img_msg)

#     def check_conditions(self, index):
#         # check if the obj was marked as "chased" or "lost" if yes return true because we dont want to inspect it again
#         if self.objectsDict[index] == 'chased':
#             return True
#         # check if the time to inspect was already over or if we lost the obj (this means that we dont had any visual detection 
#         #of the obj that we are inspect) across 30 conscutive frames
#         if self.timeToGoToObject == 0:
#             # reset the counter
#             self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT
#             self.objectsDict[index] = 'chased'
#             return True
#         # TODO: Change this number in the config file
#         if self.framesWithoutObject > 30:
#             # reset the counter
#             self.framesWithoutObject = 0
#             self.objectsDict[index] = 'lost'
#             # reset the counter
#             self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT
#             return True
#         return False

#     def draw_all_boxes(self, frame, out_boxes, out_scores, out_classes, ids):

#         # set colors of the detection boxes
#         colors = np.random.uniform(0, 255, size=(len(out_boxes), 3))

#         for i in range(len(ids)):
#             # extract the bounding box coordinates
#             x, y, w, h = out_boxes[i]
#             # get lable of the object
#             label = str(out_classes[i])
#             # get confidence of the detection
#             confidence = str(round(out_scores[i], 2))
            
#             # Get the if of the object
#             ID = str(ids[i])

#             # get color
#             color = colors[i]
#             # draw a bounding box rectangle and lable on the frame
#             cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
#             cv2.putText(frame, label + " "+ID+ " " + confidence, (x, y+20), self.font, 1, (0, 0, 0), 3)
    
#     def display_info_of_chosen_box(self, frame, x, y, w, h):

#         # Calculate the midle point of the object
#         x_medium = int((x + x+w) / 2)
#         y_medium = int((y + y+h) / 2)

#         # Draw a + in the middle point of the object found
#         cv2.line(frame , (x_medium, y_medium-1) , (x_medium, y_medium+1), (0, 255, 0), 1)
#         cv2.line(frame , (x_medium-1, y_medium) , (x_medium+1, y_medium), (0, 255, 0), 1)

#         # Draw a + on top of the optical center
#         cv2.line(frame , (self.optical_center[0], self.optical_center[1]-1) , (self.optical_center[0], self.optical_center[1]+1), (0, 0, 255), 1)
#         cv2.line(frame , (self.optical_center[0]-1, self.optical_center[1]) , (self.optical_center[0]+1, self.optical_center[1]), (0, 0, 255), 1)

#         # Draw the object's translation vector
#         cv2.line(frame , (x_medium, y_medium) , (self.optical_center[0], self.optical_center[1]), (255, 0, 0), 1)

#         # Display the error in meters
#         # Convert the midle point of the object in inertial coordinates relative to camera
#         X_medium = ((x_medium - self.principalPoint[0])*self.altitude) / self.focalDistance[0]
#         Y_medium = ((y_medium - self.principalPoint[1])*self.altitude) / self.focalDistance[1]
#         # Calculate error in meters, NOTE: the optical_center in the inertial frame is the origin
#         error_meters = norm(np.array([X_medium, Y_medium]))
#         # Display the error
#         text = "ERROR(meters): {:.2f}".format(error_meters)
#         cv2.putText(frame, text, (10, 50), self.font, 1, (0,0,0), 1)


#     @staticmethod
#     def convert_results_to_message(out_boxes, out_scores, out_classes, optical_center, lostObject, ids, imageArea, objVel):
        
#         # Create the message type
#         msgs = DetectionResults()
#         # add to the message the pixel coordinates of each image
#         msgs.opticalCenter = optical_center
#         # add to the message the area of the image
#         msgs.imageArea = imageArea
#         # add to the message the information about if we lost the object or not
#         msgs.lostObject = lostObject
#         # add information about the object in the message
#         msgs.out_class = out_classes
#         msgs.out_ids = ids
#         msgs.out_score = out_scores
#         msgs.location = out_boxes
#         msgs.obj_velocity = objVel
        
#         return msgs

#     def State_callback(self, data):
#         pass

#     def get_last_mission_callback(self, data):
#         pass
    
#     def get_mission_Flag_callback(self, data):
#         pass
        

# def main():

#     detection = DetectionNode()

#     # +.+ Going into spin; let the callbacks do all the magic 
#     rospy.spin()

# if __name__ == '__main__':
#     main()

'''
CODIGO DO DAVID
'''
#!/usr/bin/env python

"""
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
import rosnode
from detection_algorithms.david_object_detector import ObjectDetector
from detection_algorithms.KalmanFilter import KalmanFilter
from std_msgs.msg import Int32MultiArray, Bool, Int8, Float64, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from detection.msg import DetectionResults
import numpy as np
import time
import os
from uuv_sensor_ros_plugins_msgs.msg import DVL
from auv_msgs.msg import NavigationStatus
from numpy.linalg import norm
from path_following.srv import ResetVT

# Defining how many time we want to stay hovering the object
MAX_TIME_TO_GO_TO_OBJECT = 1500 #seg
class DetectionNode:
    def __init__(self):
		
        # Start the FPS Timer
        self.starting_time = time.time()
        # instanciate the counter of frame_ids
        self.frame_ids = 0
        # init the detection node
        rospy.init_node('detection_node')
		
        self.loadParams()
        # instanciate the ObjectDetector object
        self.detector = ObjectDetector(self.config_path, self.weights_path, self.classes_path, self.score_threshold, 
                self.input_size, self.principalPoint, self.focalDistance)
        

        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()
        return

    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for DetectionNode')
        # Subscribe camera
        rospy.Subscriber(self.vehicle+'/bluerov_heavy0/camera/camera_image', Image, self.image_update_callback, queue_size=5)
        # Subscribe the topic Flag
        rospy.Subscriber(self.vehicle+'/Flag', Int8, self.get_mission_Flag_callback, queue_size=1)
        # Subscribe the velocity of the robot
        rospy.Subscriber(self.vehicle+'/nav/filter/state', NavigationStatus, self.State_callback, queue_size=1)
        # Subscribe the topic that store the mission
        rospy.Subscriber(self.vehicle+'/addons/Mission_String', String, self.get_last_mission_callback, queue_size=1)
        # Subscribe the topic that store the Gamma value
        rospy.Subscriber(self.vehicle+'/Gamma', Float64, self.get_gamma_callback, queue_size=1)
    
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for DetectionNode')
        # Pub the detection
        self.detection_image_pub = rospy.Publisher('/detection/object/detection_visualization/', Image, queue_size=1)
        # Pub detection info
        self.detection_results_pub = rospy.Publisher('/detection/object/detection_result', DetectionResults, queue_size=1)
        # Pub Flag topic
        self.Flag_pub = rospy.Publisher(self.vehicle+'/Flag', Int8, queue_size=1)
        # Pub the yaw ref 
        self.refYaw_pub = rospy.Publisher(self.vehicle+'/ref/yaw', Float64, queue_size=5)
        # Pub into the Mission_String
        self.lastMission_pub = rospy.Publisher(self.vehicle+'/addons/Mission_String', String, queue_size=1)
        # Pub to tranmit the detection object
        self.detectedObject_pub = rospy.Publisher('/image_to_transmit', Image, queue_size=1)
    
    def initializeServices(self):
        rospy.loginfo('Initializing Services for DetectionNode')
        
        self.set_VT_srv = rospy.ServiceProxy(self.vehicle+'/ResetVT', ResetVT)
        
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 15)
        self.current_frame = None
        self.bridge = CvBridge()

        rospy.loginfo("Object Detection Initializing")
        rospy.loginfo("Yolo V3")
            
        self.visualize = rospy.get_param('~visualization')
        self.config_path = rospy.get_param('~config_path')
        self.weights_path = rospy.get_param('~weights_path')
        self.classes_path = rospy.get_param('~classes_path')
        self.score_threshold = rospy.get_param('~score_threshold')
        self.input_size = (416, 416) #(512, 512)
        self.principalPoint = rospy.get_param('~principalPoint')
        self.focalDistance = rospy.get_param('~focalDistance')
        
        self.Idle_flag = rospy.get_param('~Idle_flag')
        self.Mission_flag = rospy.get_param('~Mission_flag')
        self.Vision_flag = rospy.get_param('~Vision_flag')
        self.WP_flag = rospy.get_param('~WP_flag')
        self.vehicle = rospy.get_param('~Vehicle') 
        
        # set font of the text
        self.font = cv2.FONT_HERSHEY_PLAIN
        # Boolean that said if we lost the target or not, if yes we need to reset the cumulativeError
        self.lostObject = True
        # Instanciate the value for Flag mission
        self.Flag = 0
        # counter of number of frames we dont have a detection
        self.framesWithoutObject = 0
        # Creat a dictionary to the ID of the object and the state of the object (chasing, chased, lost, canceled)
        self.objectsDict = {}
        # Variable that indicate wich is the object we are chasing
        self.idObj = None
        # Instanciate the altitude value
        self.altitude = 4.5
        # Variable to store the last time
        self.lastTime = rospy.get_rostime().to_sec()
        self.dt = 0
        self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT # segundos
        self.oldYaw = None
        self.lastMission = None
        self.lastFlag = None
        self.KalmanFilter = None
        self.objVel = [0, 0]
        self.img_croped = False
        self.OnceFlag = 0
        self.gamma_value = 0.0

    def image_update_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e
        
        # Add blur to the image, to simulate water disturbances
        #cv_image = cv2.blur(cv_image, (5,5))

        # increment the number of the frames
        self.frame_ids += 1

        # check if exist a image
        if cv_image is None: 
            return

        # If we are in the WP mission we dont want to detect
        if self.Flag == self.WP_flag:
            # Reset the KalmanFilter
            self.KalmanFilter = None 
            # Reset the obj vel
            self.objVel = [0, 0]
            if self.visualize:
                self.pub_frame(cv_image, False, None, None, False, None)
            return

        # run the detector
        frame, out_boxes, out_scores, out_classes, self.optical_center, ids, self.imageArea = self.detector.detect_object(cv_image)
        
        # in case of we dont have any detection and at the same time we dont have detected any object yet, this means that our 
        # Kalman Filter is not isinstanced
        if not out_classes and self.KalmanFilter is None:
            if self.visualize:
                self.pub_frame(frame, False, out_boxes, out_scores, out_classes, ids)
            return
        
        # check if any detection occurred
        if out_classes:
            rospy.loginfo("Detection MADE")
            # get the id of the object with highest confidence
            index = self.find_object_with_more_confidence(out_scores)
                
            # get the centre of the object
            self.index_box = out_boxes[index]
            self.index_score = out_scores[index]
            self.index_class = out_classes[index]
            self.width = self.index_box[2]
            self.height = self.index_box[3]
            self.idObj = ids[index]

            # check if the object is a new detected object
            if len(self.objectsDict) == 0 or self.idObj not in self.objectsDict:

                # Add the obj into the dictionary
                self.objectsDict[self.idObj] = 'chasing'

                # Initialize the Kalman Filter with the center of the object
                self.KalmanFilter=KalmanFilter(self.index_box[0]+int(self.width/2),self.index_box[1]+int(self.height/2), 1, 0.1, 0.1)

                kf_state_pred = np.matrix([[self.index_box[0]+int(self.width/2)], [self.index_box[1]+int(self.height/2)], [0], [0]])
            
            # check if the object was already detected
            elif self.idObj in self.objectsDict:
                # Reset img_croped flag
                self.img_croped = False
                # check conditions to stop stop chasing or inspect
                if self.check_conditions(self.idObj):
                    # Activate the flag to reset the cumulative error in the IBVS controller
                    self.lostObject = True
                    # Reset the KalmanFilter
                    self.KalmanFilter = None
                    # Reset the obj vel
                    self.objVel = [0, 0]
                    # return to the last mission if exist
                    if self.lastFlag == self.Mission_flag:
                        
                        if self.visualize:
                            self.pub_frame(frame, False, out_boxes, out_scores, out_classes, ids)
                        
                        # if we already sent the last mission
                        if self.Flag == self.Mission_flag:
                            return
                        # Take a shot of the object, crop and save it in a specific folder
                        # do this just once per object
                        if self.img_croped == False:
                            # Set img_croped flag to True
                            self.img_croped = True
                            # Crop the image according to the boundering box
                            
                            # save the image as jpg
                            # self.imageTools.saveImageAsJPG(crop_img, self.idObj, os.path.expanduser('~/CapturedImages'))
                            
                            # Pub the detect_object to be transmited
                            self.detectedObject_pub.publish(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))
                    
                        # Pub the last mission, for the uav can return to it
                        self.lastMission_pub.publish(self.lastMission)
                        # set the correct value for gamma - avoiding to be stuck
                        self.set_VT_srv(self.gamma_value)
                        
                        return
                    # return to idle if there is no last mission
                    else:
                        # Take a shot of the object, crop and save it in a specific folder
                        # do this just once per object
                        if self.img_croped == False:
                            # Set img_croped flag to True
                            self.img_croped = True
                            
                            # we are not in a mission, so we publish 0 that is available to accept orther mission
                            self.Flag_pub.publish(self.Idle_flag)
                        # NOTE: if we return to idle and we have a detection we hold the position on top of the 
                        #obj based on the image detection, so we continue calling the visual controller
                
                # Another safety check
                if self.KalmanFilter is None:
                    return

                # Do an Kalman Filter PREDICT
                # Convert the total velocity of the robot from m/s to px/s in the camera plan
                self.imgVel = self.body_vel_to_image_vel(self.bodySway, self.bodySurge)
    
                # Calculate the time that passed between the last iteration and this one
                now = rospy.get_rostime().to_sec()
                # calculate the dt
                image_dt = now - self.lastTime
                # update the lastTime
                self.lastTime = now

                # Do the predict
                # According the velocity of the robot predict the next position of the target and its velocity
                # The predicted velocity will be given in px/s
                kf_state_pred = self.KalmanFilter.predict(image_dt, -self.imgVel.item(0), self.imgVel.item(1))

                # Do an Kalman Filter UPDATE
                kf_state_pred = self.KalmanFilter.update(self.index_box[0]+int(self.width/2), self.index_box[1]+int(self.height/2))
                        
                # reset the number of frames without an update
                self.framesWithoutObject = 0

        # In case that we dont have any detection
        else:
            # check conditions to stop stop chasing or inspect
            if self.check_conditions(self.idObj):
                # Activate the flag to reset the cumulative error in the IBVS controller
                self.lostObject = True
                # Reset the KalmanFilter
                self.KalmanFilter = None
                # Reset the obj vel
                self.objVel = [0, 0]
                # return to the last mission if exist or for IDLE
                if self.lastFlag == self.Mission_flag:
                    # Pub the last mission, for the uav can return to it
                    self.lastMission_pub.publish(self.lastMission)
                    # set the correct value for gamma - avoiding to be stuck
                    self.set_VT_srv(self.gamma_value)
                    # self.Flag_pub.publish(self.Idle_flag)
                    if self.visualize:
                        self.pub_frame(frame, False, out_boxes, out_scores, out_classes, ids)
                    return
                else:
                        # we are not in a mission, so we publish 0 that is available to accept orther mission
                        self.Flag_pub.publish(self.Idle_flag)
            
            # Another safety check
            if self.KalmanFilter is None:
                return

            # Check if the dictionary is not empty, which means that we already has a detection
            if len(self.objectsDict) != 0:
                # Do an Kalman Filter PREDICT
                # Convert the total velocity of the robot from m/s to px/s in the camera plan
                self.imgVel = self.body_vel_to_image_vel(self.bodySway, self.bodySurge)
    
                # Calculate the time that passed between the last iteration and this one
                now = rospy.get_rostime().to_sec()
                # calculate the dt
                image_dt = now - self.lastTime
                # update the lastTime
                self.lastTime = now

                # increment framesWithoutObject
                self.framesWithoutObject += 1
            
                # Do the predict
                # According the velocity of the robot predict the next position of the target and its velocity
                # The predicted velocity will be given in px/s
                kf_state_pred = self.KalmanFilter.predict(image_dt, -self.imgVel.item(0), self.imgVel.item(1))
                    

        # Update the box location of the object
        self.predLoc = [int(kf_state_pred.item(0)-self.width/2), int(kf_state_pred.item(1)-self.height/2), self.width, self.height]

        # Calculate the obj velocity
        objVel_aux = self.imgVel_to_bodyVel(kf_state_pred.item(2), kf_state_pred.item(3))
        
        # Save the object velocity in an array
        self.objVel = [objVel_aux.item(1), -objVel_aux.item(0)]

        # Change the mission Flag to 9
        if self.Flag != self.Vision_flag:
            if self.Flag == self.Idle_flag and self.OnceFlag == 0:
                self.oldYaw = round(self.yaw,2)
                self.OnceFlag = 1

            elif self.Flag != self.Idle_flag and self.Flag != self.WP_flag:
                self.lastFlag = self.Flag
                self.Flag_pub.publish(self.Vision_flag)
                self.oldYaw = round(self.yaw,2)
                self.OnceFlag = 0
        
        # Pub the yaw ref
        self.refYaw_pub.publish(self.oldYaw)

        # Plublishing the message with the detection to activate the IBVS Controller
        self.detection_results_pub.publish(self.convert_results_to_message(self.predLoc, self.index_score, self.index_class,
            self.optical_center, self.lostObject, 0, self.imageArea, self.objVel))


        # Reset lostObject
        self.lostObject = False
        
        # if we are in a mission and the visual_servoing_node is running 
        if self.Flag != self.Idle_flag and rosnode.rosnode_ping('/visual_servoing_node', 1, False):
            # Decrement timeToGoToObject
            self.timeToGoToObject -= 1

        if self.visualize:
            self.pub_frame(frame, True, out_boxes, out_scores, out_classes, ids)


    def find_object_with_more_confidence(self, out_scores):
        for i in range(len(out_scores)):
            index = np.argmax(out_scores)
        return index

    def check_conditions(self, index):
        # check if the obj was marked as "chased" or "lost" if yes return true because we dont want to inspect it again
        if self.objectsDict[index] == 'chased':
            return True
        # check if the time to inspect was already over or if we lost the obj (this means that we dont had any visual detection 
        #of the obj that we are inspect) across 30 conscutive frames
        if self.timeToGoToObject == 0:
            # reset the counter
            self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT
            self.objectsDict[index] = 'chased'
            return True
        # TODO: Change this number in the config file
        if self.framesWithoutObject > 30:
            # reset the counter
            self.framesWithoutObject = 0
            self.objectsDict[index] = 'lost'
            # reset the counter
            self.timeToGoToObject = MAX_TIME_TO_GO_TO_OBJECT
            return True
        return False

    def pub_frame(self, frame, existPredict, out_boxes, out_scores, out_classes, ids):
        # Calculate FPS
        elapsed_time = time.time() - self.starting_time
        fps = self.frame_ids / elapsed_time
        
        if out_classes:
            self.draw_all_boxes(frame, out_boxes, out_scores, out_classes, ids)

        if existPredict:
            # draw the predicted box
            cv2.rectangle(frame, (self.predLoc[0],self.predLoc[1]), 
                    (self.predLoc[0]+self.width, self.predLoc[1]+self.height), (255,0,0), 2)

            # display info of chosen box
            self.display_info_of_chosen_box(frame, self.predLoc[0], self.predLoc[1],
                    self.width, self.height)
           
        if self.idObj is not None:
            # Object State
            text = "Obj: " + str(self.idObj) + " -> State: " + str(self.objectsDict[self.idObj])
            cv2.putText(frame, text, (10, 110), self.font, 1, (0,0,0), 1)

        # Display fps
        text = "FPS: {:.2f}".format(fps)
        cv2.putText(frame, text, (10, 30), self.font, 1, (0,0,0), 1)
        # Display the depth
        text = "Depth(meters): {:.2f}".format(self.altitude)
        cv2.putText(frame, text, (10, 70), self.font, 1, (0,0,0), 1)
        # Flag information
        text = "Flag: {}".format(self.Flag)
        cv2.putText(frame, text, (10, 90),self.font, 1, (0,0,0), 1)
        # Time to Inspect
        text = "Time left: " + str(self.timeToGoToObject)
        cv2.putText(frame, text, (10, 130), self.font, 1, (0,0,0), 1)
        # Yaw information
        text = "RefYaw: " + str(self.oldYaw) + " | Yaw: " + str(round(self.yaw,2)) 
        cv2.putText(frame, text, (10, 150), self.font, 1, (0,0,0), 1)
        # Body Speed information
        text = "Body Vel [x,y]: [" + str(round(self.bodySurge,2)) + "," + str(round(self.bodySway,2)) + "]"
        cv2.putText(frame, text, (10, 170), self.font, 1, (0,0,0), 1)
        # Obj Speed information
        text = "OBJ Vel [x,y]: [" + str(round(self.objVel[0],2)) + "," + str(round(self.objVel[1],2)) + "]"
        cv2.putText(frame, text, (10, 190), self.font, 1, (0,0,0), 1)
        # Frames without detection
        text = "Frames without detection: " + str(self.framesWithoutObject)
        cv2.putText(frame, text, (10, 210), self.font, 1, (0,0,0), 1)
        # Convert the frame to a img message
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        #Publish the frame
        self.detection_image_pub.publish(img_msg)
    
    def computeDesiredCorners(self, w, h):
        u_img = self.optical_center[0]
        v_img = self.optical_center[1]

        return np.array([u_img-int(w/2), v_img-int(h/2), u_img+int(w/2), v_img-int(h/2), u_img+int(w/2), v_img+int(h/2)])

    def draw_all_boxes(self, frame, out_boxes, out_scores, out_classes, ids):

        # set colors of the detection boxes
        colors = np.random.uniform(0, 255, size=(len(out_boxes), 3))

        for i in range(len(ids)):
            # extract the bounding box coordinates
            x, y, w, h = out_boxes[i]
            # get lable of the object
            label = str(out_classes[i])
            # get confidence of the detection
            confidence = str(round(out_scores[i], 2))
            
            # Get the if of the object
            ID = str(ids[i])

            # get color
            color = colors[i]
            # draw a bounding box rectangle and lable on the frame
            cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
            cv2.putText(frame, label + " " + ID + " " + confidence, (x, y+20), self.font, 1, (0, 0, 0), 3)

    def display_info_of_chosen_box(self, frame, x, y, w, h):

        #compute desired points
        desired_points = self.computeDesiredCorners(100, 100)

        # Calculate the midle point of the object
        x_medium = int((x + x+w) / 2)
        y_medium = int((y + y+h) / 2)

        # Draw a + in the middle point of the object found
        cv2.line(frame , (x_medium, y_medium-1) , (x_medium, y_medium+1), (0, 255, 0), 1)
        cv2.line(frame , (x_medium-1, y_medium) , (x_medium+1, y_medium), (0, 255, 0), 1)

        # Draw a + on top of the optical center
        cv2.line(frame , (self.optical_center[0], self.optical_center[1]-1) , (self.optical_center[0], self.optical_center[1]+1), (0, 0, 255), 1)
        cv2.line(frame , (self.optical_center[0]-1, self.optical_center[1]) , (self.optical_center[0]+1, self.optical_center[1]), (0, 0, 255), 1)

        #Draw the Reference points in the camera
        #First point
        cv2.line(frame, (desired_points[0], desired_points[1]-1),(desired_points[0], desired_points[1]+1) ,(0, 0, 255), 1)
        cv2.line(frame, (desired_points[0]-1, desired_points[1]),(desired_points[0]+1, desired_points[1]) ,(0, 0, 255), 1)
        #Second point
        cv2.line(frame, (desired_points[2], desired_points[3]-1),(desired_points[2], desired_points[3]+1) ,(0, 0, 255), 1)
        cv2.line(frame, (desired_points[2]-1, desired_points[3]),(desired_points[2]+1, desired_points[3]) ,(0, 0, 255), 1)
        #Third Point
        cv2.line(frame, (desired_points[4], desired_points[5]-1),(desired_points[4], desired_points[5]+1) ,(0, 0, 255), 1)
        cv2.line(frame, (desired_points[4]-1, desired_points[5]),(desired_points[4]+1, desired_points[5]) ,(0, 0, 255), 1)

        # Draw the object's translation vector
        cv2.line(frame , (x_medium, y_medium) , (self.optical_center[0], self.optical_center[1]), (255, 0, 0), 1)

        # Display the error in meters
        # Convert the midle point of the object in inertial coordinates relative to camera
        X_medium = ((x_medium - self.principalPoint[0])*self.altitude) / self.focalDistance[0]
        Y_medium = ((y_medium - self.principalPoint[1])*self.altitude) / self.focalDistance[1]
        # Calculate error in meters, NOTE: the optical_center in the inertial frame is the origin
        error_meters = norm(np.array([X_medium, Y_medium]))
        # Display the error
        text = "ERROR(meters): {:.2f}".format(error_meters)
        cv2.putText(frame, text, (10, 50), self.font, 1, (0,0,0), 1)

    def body_vel_to_image_vel(self, surge, sway):
        '''
        Convert the body velocity to image velocity
        '''
        # set Z as the measurement altitude
        Z = self.altitude
        
        # construct a matrix with body velocities
        body_vel = np.matrix([[surge], [sway]])
        # print("body_vel : {}\n".format(body_vel))
        
        # compute the conversion matrix
        L = np.matrix([[-1/Z, 0],[0, -1/Z]])

        aux_matrix = np.zeros((2,2))
        aux_matrix[0,0] = self.focalDistance[0]
        aux_matrix[1,1] = self.focalDistance[1]

        L = np.dot(L, aux_matrix)
        # calculate the correspondent image velocity
        img_vel = np.dot(L, body_vel)

        return img_vel

    def imgVel_to_bodyVel(self, vel_px, vel_py):
        '''
        Convert the image velocity to body velocity
        '''
        # set Z as the measurement altitude
        Z = self.altitude
        
        # construct the correspondent matrix with image velocities
        imgVel = np.matrix([[vel_px], [vel_py]])
        
        # compute the conversation matrix
        # L = np.matrix([[-Z, 0],[0, -Z]])
        L = np.matrix([[-1/Z, 0],[0, -1/Z]])

        aux_matrix = np.zeros((2,2))
        aux_matrix[0,0] = self.focalDistance[0]
        aux_matrix[1,1] = self.focalDistance[1]
        
        L = np.dot(L, aux_matrix)

        # calculate the correspondent body velocity
        bodyVel = np.dot(np.linalg.inv(L), imgVel)

        return bodyVel

    @staticmethod
    def convert_results_to_message(out_boxes, out_scores, out_classes, optical_center, lostObject, ids, imageArea, objVel):
        
        # Create the message type
        msgs = DetectionResults()
        # add to the message the pixel coordinates of each image
        msgs.opticalCenter = optical_center
        # add to the message the area of the image
        msgs.imageArea = imageArea
        # add to the message the information about if we lost the object or not
        msgs.lostObject = lostObject
        # add information about the object in the message
        msgs.out_class = out_classes
        msgs.out_ids = ids
        msgs.out_score = out_scores
        msgs.location = out_boxes
        msgs.obj_velocity = objVel
        
        return msgs

    def get_mission_Flag_callback(self, data):
       self.Flag = data.data
    
    def get_last_mission_callback(self, data):
        self.lastMission = data.data

    def get_gamma_callback(self, data):
        self.gamma_value = data.data
    
    def State_callback(self, data):
        self.bodySurge = data.body_velocity.x
        self.bodySway = data.body_velocity.y
        self.yaw = data.orientation.z
        self.yaw_rate = round(data.orientation_rate.z,2)
        self.altitude = data.altitude
    
def main():
	detection = DetectionNode()
	rospy.spin()

if __name__ == '__main__':
    main()
