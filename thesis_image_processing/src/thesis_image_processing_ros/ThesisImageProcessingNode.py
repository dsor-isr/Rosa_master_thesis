#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from thesis_image_processing_algorithms.ThesisImageProcessingAlgorithm import my_generic_sum_function
from std_msgs.msg import Int8, Bool

class ThesisImageProcessingNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('thesis_image_processing_node')

        
        """
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        """

        

        """
        @.@ Dirty work of declaring subscribers, publishers and load parameters 
        """
        self.loadParams()
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()


    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency', 10)
    

    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for ThesisImageProcessingNode')

    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for ThesisImageProcessingNode')


    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()


    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        # REMOVE pass and do your magic
        pass
            


def main():

    thesis_image_processing = ThesisImageProcessingNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
