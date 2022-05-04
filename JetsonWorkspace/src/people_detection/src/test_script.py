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
                publishROS = True, peopleDetector = True, publishBB = False, 
                test_name = "Testname"):
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
        self.testName = test_name
        self.csvCreated = False # Used for ensuring that header is written when creating csv

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

    def csvCreation(self, containsperson, detection , angle, distance, x, y):
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
            left=detection[0].Left
            right=detection[0].Right
            top=detection[0].Top
            bottom=detection[0].Bottom
            
            if self.csvCreated: #test if csv is already created
                with open(self.testName + ".csv", 'a') as csvfile:
                    # creating a csv writer object
                    csvwriter = csv.writer(csvfile)

                    # writing the fields
                    csvwriter.writerow([1, self.timestamp.to_nsec(), framerate, angle, distance, x, y, left, top, right, bottom, self.count])
            else:
                 with open(self.testName + ".csv", 'a') as csvfile:
                    # creating a csv writer object
                    csvwriter = csv.writer(csvfile)

                    # writing the fields
                    csvwriter.writerow(["detection", "timestamp", "framerate", "angle", "distance", "x-coord", "y-coord", "BBleft", "BBtop", "BBright", "BBbottom", "iteration"])
                    csvwriter.writerow([1, self.timestamp.to_nsec(), framerate, angle, distance, x, y, left, top, right, bottom, self.count])
                    self.csvCreated = True

            angle = np.degrees(angle)

        else:
            if self.csvCreated: #test if csv is already created
                with open(self.testName + ".csv", 'a') as csvfile:
                    # creating a csv writer object
                    csvwriter = csv.writer(csvfile)

                    # writing the fields
                    csvwriter.writerow([0, self.timestamp.to_nsec(), framerate, None, None, None, None, None, None, None, None, self.count])
            else:
                with open(self.testName + ".csv", 'a') as csvfile:
                    # creating a csv writer object
                    csvwriter = csv.writer(csvfile)

                    # writing the fields
                    csvwriter.writerow(["detection", "timestamp", "framerate", "angle", "distance", "x-coord", "y-coord", "BBleft", "BBtop", "BBright", "BBbottom", "iteration"])
                    csvwriter.writerow([0, self.timestamp.to_nsec(), framerate, None, None, None, None, None, None, None, None, self.count])
                    self.csvCreated = True      

        self.latestTimeStamp=rospy.Time.now()###should be the last thing that happens
        
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
                ##Assuming semi static people
                x= np.cos(angle) * distance
                y= np.sin(angle) * distance 
                person.position.x = x # calculate cartesian coordinates
                person.position.y = y
                person.position.z = 0

                if not deltat==0:#aproximate velocity based on the last two positions and their time difference
                    #works only for one person in the scene otherwise use hungarian algorithm or simillar
                    person.velocity.x = (x-self.lastperson.position.x)/deltat
                    person.velocity.y = (y-self.lastperson.position.y)/deltat
                    person.velocity.z = 0
                else:#first detection cannot have velocities yet since it is lacking behind
                    person.velocity.x = 0
                    person.velocity.y = 0
                    person.velocity.z = 0

                person.reliability = detection.Confidence
                persons.append(person)
                self.lastperson=person#safe last person detection
        
        self.csvCreation(bool(len(persons)>0), peopledetections, angle, distance, x, y)#give the current state to the csv
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
        
    def rosBBmsg(self, detections):
        """
        Parameters
        ----------
        detections : detectNet.Detection[]
            an array of all detections
        """

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

if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    testName = "test_"+str(sys.argv[1])

    rospy.init_node('people_detection')
    r = rospy.Rate(10) # 10hz

    detector = PeopleLocaliser(test_name=testName)

    # while not rospy.is_shutdown():
    detector.count = 0
    while (detector.count<100):
        if (detector.findPeople()): detector.count=detector.count+1
        #r.sleep()
        rospy.loginfo("Iteration " + str(detector.count))
