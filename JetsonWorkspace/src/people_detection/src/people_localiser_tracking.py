#!/usr/bin/env python

from cmath import sin
from turtle import right
import rospy
import rospkg
import jetson.inference
import jetson.utils

import sys

from people_detection.msg import BoundingBox,BoundingBoxes
from people_msgs.msg import People,Person
from sensor_msgs.msg import Image,CameraInfo

from cv_bridge import CvBridge
import cv2
import numpy as np
import csv

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
    ###
        #https://github.com/IntelRealSense/librealsense/issues/2141
        #hfov   54.732
        #vfov   42.4115
    ###.
    def __init__(self, networkname = "ssd-mobilenet-v2",
                resolution = (640,480), threshold = 0.5, HFOV = np.radians(54.732),
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


        self.kf = cv2.KalmanFilter(4,2) # Kalman filter with states[x,y,dx,dy] and measurements x,y
        self.kf.processNoiseCov=np.array([  [1e-2, 0, 0, 0],
                                            [0, 1e-2, 0, 0],
                                            [0, 0, 5, 0],
                                            [0, 0, 0, 5]],np.float32) # noise values from https://github.com/Myzhar/simple-opencv-kalman-tracker/blob/master/source/opencv-kalman.cpp
        self.kf.measurementNoiseCov=np.eye(1e-1, np.float32)
        self.state=np.zeros(4, np.float32)
        self.ms = np.zeros(2, np.float32)



        self.HFOV = HFOV 
        self.threshold = threshold
        self.publishROSmsg = publishROS
        self.publishBB = publishBB
        self.peopleDetector = peopleDetector
        self.detections = None
        self.timestamp = rospy.Time.now()
        self.latestTimeStamp = rospy.Time.now()
        self.Counter = 0
        self.peopleKeepTime = 5 ##Seconds How long to keep detected people after not detecting them anymore
        self.bridge = CvBridge()
        self.rgb=None
        self.depth=None
        self.lastperson = Person()
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
            self.depth = np.array(depth, dtype=np.float32)*0.001
        except:
            pass
    
    def findPeople(self):
        """
        Main function for detection and publishing people
        """

        self.timestamp = rospy.Time.now()
        frameid = 'base_link' #Stand-in until added to capture image
        colour, depth = self.captureImages()

        #Check if there is images available
        if not isinstance(colour, type(None)) and not isinstance(depth, type(None)):
            if self.networkname == "ssd-mobilenet-v2":
                people = self.detectSSD(colour, depth)
        
            self.rosPeoplemsg(people, frameid, self.timestamp)
            return True
        else: return False
        
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

        width = right - left
        height = bottom - top
 
        #relative to the image center such that the distances are postive going left and upwards according to REP
        centreX = (self.resolutionX/2) - ((right + left)/2)
        centreY = (self.resolutionY/2) - ((top + bottom)/2)

        #Length of the distance to the virtual image plane
        Id = (self.resolutionX/2)/np.tan(self.HFOV/2)
        #Length of the hypothenuse going towards the bb on the y=centrey plane
        Idx = np.sqrt((Id**2) + (centreX**2))

        #angle between Idx and the line going from the camera to the centre of the bb
        delta = np.arctan2(centreY,Idx)

        #Angle between idx and ID
        gamma = np.arctan2(centreX,Id)

        ##get distances of depth image assuming same resolution and allignment relative to bounding box coordinates
        distBox = depth[ int(top + (height/4)):int(bottom - (height/2)),int(left + (width/4)):int(right - (width/4))]
        distBox = distBox.flatten()

        distBox = np.delete(distBox, np.argwhere(distBox == 0))

        distance = np.nanmedian(distBox)

        #Projection to horizontal plane happening here
        distance = distance*np.cos(delta)
        #Output in polar coordinates such that angles to the left are positive and angles to the right are negative
        #Distance Forward is positiv backwards not possible
        return distance, gamma

    def captureImages(self):    
        """
        -------
        Assign current images for detection
        """
        return self.rgb, self.depth

    def getClass(self, Index):
        return self.labels[Index]#in case there is no function otherwise overwrite

    def initdetectNet(self):
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

        cudaimage = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA).astype(np.float32)#converting the image to a cuda compatible image
        cudaimage = jetson.utils.cudaFromNumpy(cudaimage)

        detections = self.net.Detect(cudaimage, cudaimage.shape[1], cudaimage.shape[0])#returning the detected objects
        persons = []
        distance = None
        angle = None
        x = None
        y = None
        peopledetections=[]
        for detection in detections:
            if int(detection.ClassID) == 1:#Only do this if its a person
                peopledetections.append(detection)

                person = Person()

                distance, angle = self.findPosition(detection.Top, detection.Left, detection.Right, detection.Bottom, depth)

                deltat=(self.timestamp.to_sec()-self.latestTimeStamp.to_sec())
                

                person.name = "Bob"
                
                x= np.cos(angle) * distance
                y= np.sin(angle) * distance 

                #Tracking measurement
                self.ms[0] = x
                self.ms[1] = y
                self.kf.correct(self.ms)

                if not deltat==0:#aproximate velocity based on the last two positions and their time difference between the last two predictions TODO try to use other states of kf instead
                    #works only for one person in the scene otherwise use hungarian algorithm or simillar
                    self.kf.transitionMatrix[0,2] = deltat  #update deltat of kalman filter to the duration of this detection needs to be done since we dont run at fixed framerate
                    self.kf.transitionMatrix[1,4] = deltat
                    self.state = self.kf.predict()          #predict the new cartesian position of the person
                    person.position.x = self.state[0]
                    person.position.y = self.state[1] 
                    person.position.z = 0
                    
                    person.velocity.x = (x-self.lastperson.position.x)/deltat
                    person.velocity.y = (y-self.lastperson.position.y)/deltat
                    person.velocity.z = 0
                else:
                    #first detection cannot have velocities yet since it is lacking behind
                    #cannot predict for first detection so set measurement
                    person.position.x = self.ms[0]
                    person.position.y = self.ms[1] 
                    person.position.z = 0
                    
                    person.velocity.x = 0
                    person.velocity.y = 0
                    person.velocity.z = 0

                person.reliability = detection.Confidence
                persons.append(person)
                self.lastperson=person#safe last person detection
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

        if len(persons)>0: #If we detected at least one person update the timestamp of the latest succesful detection
            self.latestTimeStamp=self.timestamp
        ### We want to publish persons if we detect at least one
        ### Or we want to publish an empty people message if the detections have been of len 0 for more the people keep time
        if  len(persons)>0 or (self.timestamp.to_sec()-self.latestTimeStamp.to_sec())>self.peopleKeepTime:
            people = People()
            people.people = persons
            people.header.stamp = timestamp#we might want to make this the time of when the camera recorded the people
            people.header.frame_id = frameid

            self.peoplePub.publish(people)

if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()
    rospy.init_node('people_localiser')
    detector = PeopleLocaliser()

    while not rospy.is_shutdown():
        detector.findPeople()
