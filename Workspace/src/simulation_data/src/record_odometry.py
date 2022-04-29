#!/usr/bin/env python3
import sys
import csv

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print ("heard "+str(data.pose.pose.position.x))
    #print ("heard "+str(data.header.stamp.secs))
    csvwriter.writerow([str(data.header.stamp.secs), str(data.header.stamp.nsecs), str(data.pose.pose.position.x), str(float(data.pose.pose.position.y)+10.0)])
    print (str(data.header.stamp.secs) + str(data.header.stamp.nsecs) + str(data.pose.pose.position.x) + str(float(data.pose.pose.position.y)+10.0))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odometry_listener', anonymous=True)
    rospy.Rate(1)
    #rospy.Subscriber("odom", String, callback)
    rospy.Subscriber('odom', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    csvFileName = str(sys.argv[1])

    makeCsvFileSuccess = False
    with open(csvFileName, 'a') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)

        print ("Successfully created "+csvFileName)
        makeCsvFileSuccess = True

        if makeCsvFileSuccess:
            print("Start odom_listener")
            csvwriter.writerow(["secs", "nsecs", "x", "y"])
            listener()
        else:
            print ("Abort!")