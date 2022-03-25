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

    def __init__(self, networkname = "ssd-mobilenet-v2", resolution = (1280, 720), threshold = 0.5, publishROS = True,  peopleDetector = True, publishBB = False):
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
        self.resolution = resolution
        self.threshold = threshold
        self.publishROSmsg = publishROS
        self.publishBB = publishBB
        self.peopleDetector = peopleDetector
        self.detections = None
        self.labels = {0: "person"} #TODO: add file for labels


        #Setup of Realsense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.profile = self.pipeline.start(config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

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
        """
        Uses the Realsense pipeline to wait for and align colour and depth image

        Return
        -------
        colour_image : np_array
            output colour image aligned with depth image
        depth_image : np_array
            output depth image aligned with colour image
        """
        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        colour_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        colour_image = np.asanyarray(colour_frame.get_data())
        
        return colour_image, depth_image

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

    def __del__(self):
        print("Detector destroyed")
        self.pipeline.stop()