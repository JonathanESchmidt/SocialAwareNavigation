#!/usr/bin/env python3

import rospy
import rospkg
import jetson.inference
import jetson.utils

import argparse
import sys

from people_detection.msg import BoundingBox,BoundingBoxes

import cv2, numpy
#from YOLOv4 import tfYOLO

#Just a wrapper so we can make derivates if we need to use other architectures
class detector:
   def __init__(self,networkname,publishROS, threshold):
      self.net=None
      self.networkname=networkname
      self.threshold=threshold
      self.publishROSmsg=publishROS
      self.detections=None
      self.labels={0:"person"}
      if self.publishROSmsg: #only init publisher if necessary
         self.pub = rospy.Publisher('people', BoundingBoxes, queue_size=10)

   def getClass(Index):
      return self.labels[Index]#in case there is no function otherwise overwrite

   def detect(image):#this is very specific to the network architecture so pass
      pass

   def rosmsg(detections):#This is the general structure assuming a DetectNet like detections object overwrite if necessary
      Boxes=BoundingBoxes()
      Box=BoundingBox()
      Index=0
      for detection in detections:
         Box.Class=str(self.getClass(detection.ClassID))
         Box.probability=float(detection.Confidence)
         Box.id=int(detection.ClassID)
         Box.xmin=int(detection.Left)
         Box.xmax=int(detection.Right)
         Box.ymin=int(detection.Top)
         Box.ymax=int(detection.Bottom)

         Boxes.bounding_boxes.append(Box)
         Index+=1

      Boxes.header.stamp=rospy.Time.now()
      self.pub.publish(Boxes)

class DetectNET(detector):
   def __init__(self,networkname="ssd-mobilenet-v2",publishROS=False, threshold=0.5):
         super().__init__(networkname, publishROS, threshold) #inheriting everything from the parent class
         self.net = jetson.inference.detectNet(networkname, sys.argv, threshold)

   def getClass(Index):
       return self.GetClassDesc(Index)

   def detect(image):#expecting opencv image
      image = cv2.cvtColor(input, cv2.COLOR_RGB2RGBA).astype(np.float32)
      image = jetson.utils.cudaFromNumpy(image)
      detections = self.net.Detect(image, input.shape[1], input.shape[0])
      if self.publishROSmsg: self.rosmsg(detections)
      return detections


if __name__ == "__main__":
   #get path of the weights from rospkg so we can use it relative
   rospack = rospkg.RosPack()

   
   rospy.init_node('people_detection')
   r = rospy.Rate(10) # 10hz

   det=DetectNET()
   vid = cv2.VideoCapture(0)#using opencv to get camera stream might produce some overhead idk

   while not rospy.is_shutdown():
      ret, frame = vid.read()

      detections=det.detect(frame)

      #r.sleep() can be re-introduced if the detector runs too fast :)