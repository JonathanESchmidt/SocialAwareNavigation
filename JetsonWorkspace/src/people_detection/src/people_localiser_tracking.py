#!/usr/bin/env python

import rospy
import rospkg
import jetson.inference
import jetson.utils

import sys

from people_msgs.msg import People,Person
from sensor_msgs.msg import Image,CameraInfo

from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy import optimize
from tracker import tracker

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

        self.trackers=[]

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
        
        self.net = jetson.inference.detectNet(self.networkname, sys.argv, self.threshold)

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
        colour, depth = self.rgb, self.depth

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


        x = np.cos(gamma) * distance
        y = np.sin(gamma) * distance 
        return x, y
    def detectSSD(self, image, depth):#this is very specific to the network architecture so pass
        """
        BRIEF
        ----------
        This function runs multiobject tracking based on kalman filters and the hungarian method

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

        coordinates = []
        persons = []

        #filter out all detections that are enot human
        detections = [detection for detection in detections if detection.ClassID == 1]
        
        for j in range(len(detections)):
            #perform trigonmetry to get carrtesian coordinates and safe all relevant data in coordinates
            coordinates.append([self.findPosition(detections[j].Top, detections[j].Left, detections[j].Right, detections[j].Bottom, depth),detections[j].Confidence])

            #created costmatrix for hungarian method
            #cost matrix has a row for each detection and a column for each tracker
            if len(coordinates) > len(self.trackers):                           #if more detections than trackers exist add a row
                costmatrix=np.zeros(len(coordinates),len(coordinates))
            else:
                costmatrix=np.zeros(len(coordinates),len(self.trackers))        #if more trackers exist make cost matrix rectangular and remove using skipy function

            for i in range(len(self.trackers)):
                diffx=self.trackers[i].predict(self.timestamp)[0]-coordinates[j][0]
                diffy=self.trackers[i].predict(self.timestamp)[1]-coordinates[j][1]
                costmatrix[j,i]=np.sqrt(np.square(diffx)+np.square(diffy))      #create costmatrix based on euclidean distance to the predicted state
                
        row_ind, col_ind = optimize.linear_sum_assignment(costmatrix)       #run hungarian method

        for i in range(np.max(col_ind)-len(self.trackers)):             #append new necessary trackers
            self.trackers.append(tracker())

        for i in range(len(col_ind)):                                       #update all assigned trackers
            person = Person()
            x=coordinates[row_ind[i]][0]
            y=coordinates[row_ind[i]][1]

            state = self.trackers[col_ind[i]].update(x,y,self.timestamp)    #get updated tracker pose

            person.name = str(col_ind[i])
            person.position.x = state[0]
            person.position.y = state[1] 
            person.position.z = 0
            
            person.velocity.x = state[2]
            person.velocity.y = state[3]
            person.velocity.z = 0
            person.reliability = coordinates[2]
            persons.append(person)


        self.trackers=[tracker for tracker in self.trackers if (self.timestamp-tracker.timestamp)<self.peopleKeepTime] #remove all trackers that have no assigned people within keepTime

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

        ### We want to publish persons if we detect at least one
        ### Or we want to publish an empty people message if no valid trackers are running
        if  len(persons)>0 or len(self.trackers)==0:
            people = People()
            people.people = persons
            people.header.stamp = timestamp #TODO we might want to make this the time of when the camera recorded the people
            people.header.frame_id = frameid

            self.peoplePub.publish(people)

if __name__ == "__main__":
    rospy.init_node('people_localiser')
    detector = PeopleLocaliser()

    while not rospy.is_shutdown():
        detector.findPeople()
