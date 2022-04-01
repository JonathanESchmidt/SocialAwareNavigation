#!/usr/bin/env python3

import rospy
import rospkg

from people_msgs.msg import People,Person


if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    rospy.init_node('people_sim')
    
    peoplePub = rospy.Publisher('people', People , queue_size=10)

    r = rospy.Rate(5) # 10hz

    person = Person()
    person.name = 'bob'
    person.position.x = 1.0
    person.position.y = 1.0
    person.position.z = 0.0
    person.velocity.x = 0.1
    person.velocity.y = 0.1
    person.velocity.z = 0.0
    person.reliability = 0.9

    people = People()
    people.people = [person]
    
    people.header.frame_id = 'map'

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        #print ("current time: "+str(current_time))
        people.header.stamp = current_time

        if (person.position.y > 3) or (person.position.y < 1):
            person.velocity.y = person.velocity.y * (-1)
            person.velocity.x = person.velocity.x * (-1)
        
        person.position.x = person.position.x + person.velocity.x
        person.position.y = person.position.y + person.velocity.y


        peoplePub.publish(people)
        rospy.sleep(0.5)
