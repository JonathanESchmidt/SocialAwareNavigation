#!/usr/bin/env python

from cmath import sin
from turtle import right
import rospy
import rospkg
import jetson.inference
import jetson.utils

import argparse
import sys

from people_detection.msg import BoundingBox,BoundingBoxes
from people_msgs.msg import People,Person
from sensor_msgs.msg import Image,CameraInfo

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
                resolution = (640,480), threshold = 0.5, HFOV = np.radians(69.4),
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
        self.resolutionY = resolution[1]
        self.output = cv2.VideoWriter(
                            "Testname.avi", cv2.VideoWriter_fourcc(*'MPEG'), 30, (self.resolutionY, self.resolutionX))

        self.HFOV = HFOV 
        self.threshold = threshold
        self.publishROSmsg = publishROS
        self.publishBB = publishBB
        self.peopleDetector = peopleDetector
        self.detections = None
        self.timestamp = rospy.Time.now()
        self.latestTimeStamp = None
        self.Counter = 0
        self.peopleKeepTime = 5 ##Seconds How long to keep detected people after not detecting them anymore
        self.bridge = CvBridge()
        self.rgb=None
        self.depth=None

        self.peoplePub = rospy.Publisher('people', People , queue_size=10)
        if self.publishROSmsg: #only init publisher if necessary
            self.bbPub = rospy.Publisher('BoundingBoxes', BoundingBoxes, queue_size=10)

        if self.peopleDetector:
            self.labels = {0: "person"}
        else:
            #TODO: use the get class function for detections
            pass
        
        self.initdetectNet()

        #TODO: add system for publishing Bounding boxes

        #Definition of subscribers to the realsense topics
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        #self.rgb_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.rgb_info_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

    #def rgb_info_callback(self, info):
    #    self.resolutionX=info.width
    #    self.resolutionY=info.height

    def rgb_callback(self, rgb):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='passthrough')
        except:
            pass
    def depth_callback(self,depth):
        try:
            depth=self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            self.depth = np.array(depth, dtype=np.uint16)*0.001
        except:
            pass
    
    def findPeople(self):
        """
        Main function for detection and publishing people
        """

        self.timestamp = rospy.Time.now()
        frameid = 'baselink' #Stand-in until added to capture image
        colour, depth = self.captureImages()

        #Check if there is images available
        if not isinstance(colour, type(None)) and not isinstance(depth, type(None)):
            if self.networkname == "ssd-mobilenet-v2":
                people = self.detectSSD(colour, depth)
        
            self.rosPeoplemsg(people, frameid, self.timestamp)
        
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
        #TODO: calculate position correctly
        width = right - left
        height = bottom - top

        #relative to the image center such that the distances are postive going right and upwards
        centreX = ((right + left)/2) - (self.resolutionX/2)
        #centreY = (self.resolutionY/2) - (height/2) not necessary since we dont care about z

        ### calculate angle relative to the image center such that the angle is positive going to the right
        #   This assumes that the camera is mounted exactly in the middle of the robot
        angleX = math.atan((2*centreX*math.tan(self.HFOV/2))/self.resolutionX)


        #distBox = depth[int(centreX-(width/4)):int(centreX+(width/4)), int(centreY-(height/4)):int(centreY+(height/4))]

        ##get distances of depth image assuming same resolution and allignment relative to bounding box coordinates
        
        distBox = depth[int(left + (width/4)):int(right - (width/4)), int(top + (height/4)):int(bottom - (height/4))]
        distBox = distBox.flatten()
        distBox = np.delete(distBox, np.argwhere(distBox == 0))

        distance = np.average(distBox)

        


        return distance, angleX

    def captureImages(self):    
        """
        -------
        Assign current images for detection
        """
        return self.rgb, self.depth

    def getClass(self, Index):
        rospy.loginfo("Entered getClass")
        return self.labels[Index]#in case there is no function otherwise overwrite

    def initdetectNet(self):
        self.net = jetson.inference.detectNet(self.networkname, sys.argv, self.threshold)

    def videoCreation(self, image, containsperson, detection, angle, distance):
        """
        BRIEF
        --------------
        Image
        Bounding Box Coordinates
        Framerate
        Distance
        Angle
        Cartesian Coordinates
        Flag if contains person


        """

        framerate = 1/(rospy.Time.now().to_sec()-self.timestamp.to_sec())# 1/calculationtime
        
        if containsperson:
            left=detection.Left
            right=detection.Right
            top=detection.Top
            bottom=detection.Bottom

            cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 3)
            cv2.putText(image, f"Polar: {angle}deg, {distance}m", (100, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, f"Cartesian: {x}X, {y}Y", (200, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(image, f"FPS: {framerate}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        self.output.write(image)

        self.FPStimestamp=rospy.Time.now()###should be the last thing that happens
        
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
            person = Person()

            distance, angle = self.findPosition(detection.Top, detection.Left, detection.Right, detection.Bottom, depth)
            person.name = "Bob"
            ##Assuming semi static people
            person.velocity.x = 0
            person.velocity.y = 0
            person.velocity.z = 0

            person.position.x = np.cos(angle) * distance # calculate cartesian coordinates
            person.position.y = - np.sin(angle) * distance
            person.position.z = 0

            person.reliability = detection.Confidence
            persons.append(person)
            
        self.videoCreation(image, bool(len(person)>0), detection , np.degrees(angle), distance)#give the current state to the video
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

        if len(persons)>0: #If we detected at least one person update the timestamp of the latest succesful detection
            self.latestTimeStamp=self.timestamp
        ### We want to publish persons if we detect at least one
        ### Or we want to publish an empty people message if the detections have been of len 0 for more the people keep time
        if  len(persons)>0 or (self.timestamp.to_sec-self.latestTimeStamp.to_sec)>self.peopleKeepTime:
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
        cv2.destroyAllWindows()
        self.output.release()

if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    rospy.init_node('people_detection')
    r = rospy.Rate(10) # 10hz

    detector = PeopleLocaliser()

    while not rospy.is_shutdown():
        detector.findPeople()