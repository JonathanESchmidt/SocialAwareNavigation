#!/usr/bin/env python3

import rospy
import rospkg
import jetson.inference
import jetson.utils

import pyrealsense2 as rs

import argparse
import sys

from people_detection.msg import BoundingBox,BoundingBoxes

import cv2
import numpy as np


#Just a wrapper so we can make derivates if we need to use other architectures
class peopleLocaliser:
    """
    A class used to localise people for the ROS peoples messages
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

    def __init__(self, networkname = "ssd-mobilenet-v2", threshold = 0.5, publishROS = True,  peopleDetector = True, publishBB = False):
        """
        Parameters
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
        """

        self.net = None
        self.networkname = networkname
        self.threshold = threshold
        self.publishROSmsg = publishROS
        self.publishBB = publishBB
        self.peopleDetector = peopleDetector
        self.detections = None
        self.labels = {0: "person"} #TODO: add file for labels

        if self.publishROSmsg: #only init publisher if necessary
            self.pub = rospy.Publisher('people', BoundingBoxes, queue_size=10)

        if self.peopleDetector:
            #TODO: add the index for people
            pass
        else:
            #TODO: use the get class function for detections
            pass

        #TODO: add system for publishing Bounding boxes

    def findPeople(self):
        #TODO: add 
        pass

    def captureImages(self):
        #TODO: add realsense image stuff
        pass

    def getClass(self, Index):
        return self.labels[Index]#in case there is no function otherwise overwrite

    def detect(self, image):#this is very specific to the network architecture so pass
        #TODO: add detector stuff
        pass



    def rosPeoplemsg(selv, person):
        #TODO: add ros msg
        pass

    def rosBBmsg(self, detections):#This is the general structure assuming a DetectNet like detections object overwrite if necessary
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
        self.pub.publish(Boxes)