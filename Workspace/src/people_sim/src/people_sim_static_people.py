#!/usr/bin/env python3

import rospy
import rospkg

from BasicPerson import BasicPerson

from people_msgs.msg import People,Person

def stay_put(person):
    pass


if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    rospy.init_node('people_sim')
    
    peoplePub = rospy.Publisher('people', People , queue_size=10)

    r = rospy.Rate(5) # 10hz

    human1 = BasicPerson('bob1', 2, 1, 0, 0.0, 0.1, 0, 0.9, stay_put)
    human2 = BasicPerson('bob2', 1, 2, 0, 0.1, 0.1, 0, 0.9, stay_put)
    human3 = BasicPerson('bob3', -3, 2, 0, 0.1, 0.1, 0, 0.9, stay_put)
    human4 = BasicPerson('bob4', -3, -1, 0, 0.1, 0.1, 0, 0.9, stay_put)

    humans = [human1, human2, human3, human4]

    people = People()
    
    people.header.frame_id = 'map'

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        people.people = []
        for h in humans:
            h.update_position()
            people.people.append(h.get_person_data())

        peoplePub.publish(people)
        rospy.sleep(0.5)