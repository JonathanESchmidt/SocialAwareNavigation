#!/usr/bin/env python

from cmath import sin
import rospy
import rospkg
import jetson.inference
import jetson.utils

#import pyrealsense2 as rs

import argparse
import sys

from people_detection.msg import BoundingBox,BoundingBoxes
from people_msgs.msg import People,Person
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PeopleLocaliser():
    """
    A class used to localise people for the ROS peoples messages
    using an Intel realsense camera
    ...

    Attributes
    ----------
    network : str
        a string representing the network used 
        available networks are: "ssd-mobilenet-v2",
    threshold : float
        threshold of probability for detected objects
    publishROS : bool
        determine if the message is published to ROS
    PeopleDetector : bool
        determine if the network will only publish people
    publishBB : bool
        determine if the Bounding Boxes are published

    Methods
    -------
    TODO: Add description of methods
    """

    def __init__(self, networkname = "ssd-mobilenet-v2",
                resolution = (1280, 720), threshold = 0.5, HFOV = 91.2,
                publishROS = True, peopleDetector = True, publishBB = False):
        """
        Parameters
        ----------
        network : str
            a string representing the network used 
            available networks are: "ssd-mobilenet-v2",
        resolution : tuple(int)
            resolution to be used by the camera
        HFOV : float
            Horizontal field of view of the camera
        threshold : float
            threshold of probability for detected objects
        publishROS : bool
            determine if the message is published to ROS
        PeopleDetector : bool
            determine if the network will only publish people
        publishBB : bool
            determine if the Bounding Boxes are published
        """

        self.net = None
        self.networkname = networkname
        self.resolutionX = resolution[0]
        self.resolutiony = resolution[1]
        self.HFOV = HFOV 
        self.threshold = threshold
        self.publishROSmsg = publishROS
        self.publishBB = publishBB
        self.peopleDetector = peopleDetector
        self.detections = None
        
        #Setup of Realsense camera
        #self.pipeline = rs.pipeline()
        #self.config = rs.config()
        self.bridge = CvBridge()
        self.rgb=None
        self.depth=None


        #self.config.enable_stream(rs.stream.depth, self.resolutionX, self.resolutionY, rs.format.z16, 30)
        #self.config.enable_stream(rs.stream.color, self.resolutionX, self.resolutionY, rs.format.bgr8, 30)

        #self.profile = self.pipeline.start(self.config)
        #self.depth_sensor = self.profile.get_device().first_depth_sensor()
        #self.depth_scale = self.depth_sensor.get_depth_scale()

        #self.align_to = rs.stream.color
        #self.align = rs.align(self.align_to)

        self.peoplePub = rospy.Publisher('people', People , queue_size=10)
        if self.publishROSmsg: #only init publisher if necessary
            self.bbPub = rospy.Publisher('BoundingBoxes', BoundingBoxes, queue_size=10)

        if self.peopleDetector:
            self.labels = {0: "person"}
        else:
            #TODO: use the get class function for detections
            pass
        
        self.initdetectNet() #TODO hardcoded for now might need a switch case for different architectures

        #TODO: add system for publishing Bounding boxes

        #Definition of subscribers to the realsense topics
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

    def rgb_callback(self, rgb):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='passthrough')
        except:
            pass
    def depth_callback(self,depth):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        except:
            pass
    
    def findPeople(self):
        """
        Main function for detection and publishing people
        """
        #rospy.loginfo("Entered findPeople")

        timestamp = None
        frameid = None #Stand-in until added to capture image
        colour, depth = self.captureImages()#TODO call find depth function
        detections = None
        if not isinstance(colour, type(None)):
            if self.networkname == "ssd-mobilenet-v2":
                people = self.detectSSD(colour, depth)


        
            self.rosPeoplemsg(people, frameid, timestamp)
        
    def findPosition(self, top, left, right, bottom, depth):
        """
        Parameters
        ----------
        depth : numpy array 
            numpy array of depth aligned with the image for detections

        Return
        ----------
        distance : float
            distance to the detected person in meters

        angle : float
            angle to the centre of the bounding box of the person
        """
        rospy.loginfo("Entered findPosition")
        #TODO: add function for finding distance of people
        width = right - left
        height = bottom - top

        centreX = int(left + (width/2))
        centreY = int(top + (height/2))

        distBox = depth[int(centreX-(width/4)):int(centreX+(width/4)), int(centreY-(height/4)):int(centreY+(height/4))]

        distBox = distBox.flatten()
        distBox = np.delete(distBox, np.argwhere(distBox == 0))

        distance = np.average(distBox)

        angleX = math.atan2((2*centreX*math.tan(self.HFOV))/self.resolutionX)


        return distance, angleX


        



    def captureImages(self):
        """
        Uses the Realsense pipeline to wait for and align colour and depth image

        Return
        -------
        colour_image : np_array
            output colour image aligned with depth image
        depth_image : np_array
            output depth image aligned with colour image
        """
        
        """
        BRIEF
        -------
        this now just returns the latest depth and rgb image
        
        """



        # rospy.loginfo("Entered captureImages")
        
        # frames = self.pipeline.wait_for_frames()

        # aligned_frames = self.align.process(frames)
        # aligned_depth_frame = aligned_frames.get_depth_frame()
        # colour_frame = aligned_frames.get_color_frame()

        # depth_image = np.asanyarray(aligned_depth_frame.get_data())
        # colour_image = np.asanyarray(colour_frame.get_data())
        
        return self.rgb, self.depth

    def getClass(self, Index):
        rospy.loginfo("Entered getClass")
        return self.labels[Index]#in case there is no function otherwise overwrite

    def initdetectNet(self): #TODO this needs to be called from either INIT or from outside
        self.net = jetson.inference.detectNet(self.networkname, sys.argv, self.threshold)
        
    def detectSSD(self, image, depth):#this is very specific to the network architecture so pass
        """
        Parameters
        ----------
        image : np.array[]
            regular numpy array in bgr8 convention

        Return
        ----------
        people : people_msgs/Person[]
            array of persons
        """
        rospy.loginfo("Entered detectSSD")

        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA).astype(np.float32)#converting the image to a cuda compatible image
        image = jetson.utils.cudaFromNumpy(image)

        detections = self.net.Detect(image, image.shape[1], image.shape[0])#returning the detected objects
        persons = []

        for detection in detections:
            # TODO: fill out person object
            person = Person()

            distance, angle = self.findPosition(detection.Top, detection.Left, detection.Right, detection.Bottom, depth)

            person.position.x = np.sin(angle) * distance # calculate cartesian coordinates
            person.position.y = np.cos(angle) * distance
            person.position.z = 0
            #TODO if we introduce tracking we want to introduce velocities for the people here based on that
            persons.append(person)

        return persons

 
    def rosPeoplemsg(self, persons, frameid, timestamp):
        """
        Parameters
        ----------
        persons : people_msgs/Person[]
            an array that contains all of the detected humans,
        frameid : str
            specifies to which tf-frame the positions of the detectet people are relative to,
        timestamp : rospy.rostime.Time
            the time stamp of when the picture has been taken
        """
        rospy.loginfo("Entered rosPeoplemsg")

        people = People()
        people.people = persons
        people.header.stamp = timestamp#we might want to make this the time of when the camera recorded the people
        people.header.frame_id = frameid

        self.peoplePub.publish(people)
        
    def rosBBmsg(self, detections):
        """
        Parameters
        ----------
        detections : detectNet.Detection[]
            an array of all detections
        """

        rospy.loginfo("Entered rosBBmsg")

        Boxes = BoundingBoxes()
        Box = BoundingBox()
        Index = 0
        for detection in detections:
            Box.Class = str(self.getClass(detection.ClassID))
            Box.probability = float(detection.Confidence)
            Box.id = int(detection.ClassID)
            Box.xmin = int(detection.Left)
            Box.xmax = int(detection.Right)
            Box.ymin = int(detection.Top)
            Box.ymax = int(detection.Bottom)

            Boxes.bounding_boxes.append(Box)
            Index += 1

        Boxes.header.stamp = rospy.Time.now()
        self.bbPub.publish(Boxes)

    def __del__(self):
        print("Detector destroyed")
        self.pipeline.stop()

if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    
    rospy.init_node('people_detection')
    r = rospy.Rate(10) # 10hz

    detector = PeopleLocaliser()

    while not rospy.is_shutdown():
        detector.findPeople()