#!/usr/bin/env python3

import rospy
import rospkg

from people_detection.msg import BoundingBox,BoundingBoxes

import cv2
from YOLOv4 import tfYOLO



 
def publishDetections(model, objectType = 0):
   #Publish an array of thebounding boxes of all detected humans
   Boxes=BoundingBoxes()
   Box=BoundingBox()
   Index=0
   for (xmin, ymin, xmax, ymax), score, class_idx in zip(model.boxes, model.scores, model.classes):
      if score > 0 and class_idx == objectType: 

            Box.Class=str(model.CLASSES[class_idx])
            Box.probability=float(score)
            Box.id=int(Index)
            Box.xmin=int(xmin)
            Box.xmax=int(xmax)
            Box.ymin=int(ymin)
            Box.ymax=int(ymax)

            Boxes.bounding_boxes.append(Box)
            Index+=1

   Boxes.header.stamp=rospy.Time.now()
   pub.publish(Boxes)



if __name__ == "__main__":
   #get path of the weights from rospkg so we can use it relative
   rospack = rospkg.RosPack()
   path=rospack.get_path('people_detection')+"/src/YOLOv4/yolov4.h5"

   pub = rospy.Publisher('people', BoundingBoxes, queue_size=10)
   rospy.init_node('people_detection')
   r = rospy.Rate(10) # 10hz

   testModel = tfYOLO.tfYOLOv4(path=path)
   vid = cv2.VideoCapture(0)#using opencv to get camera stream might produce some overhead idk

   while not rospy.is_shutdown():
      ret, frame = vid.read()
      testModel.predictOPENCV(frame)
      publishDetections(testModel)

      #r.sleep() can be re-introduced if the detector runs too fast :)