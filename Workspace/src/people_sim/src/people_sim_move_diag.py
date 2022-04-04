#!/usr/bin/env python3

import rospy
import rospkg

from BasicPerson import BasicPerson

from people_msgs.msg import People,Person

def move_diagonally(person):
    if (person.position.y > 3) or (person.position.y < 1):
        person.velocity.y = person.velocity.y * (-1)
        person.velocity.x = person.velocity.x * (-1)
        
    person.position.x = person.position.x + person.velocity.x
    person.position.y = person.position.y + person.velocity.y


if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    rospy.init_node('people_sim')
    
    peoplePub = rospy.Publisher('people', People , queue_size=10)

    r = rospy.Rate(5) # 10hz

    human = BasicPerson('bob', 1, 1, 0, 0.1, 0.1, 0, 0.9, move_diagonally)

    humans = [human]

    people = People()
    
    people.header.frame_id = 'map'

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        #print ("current time: "+str(current_time))
        people.people = []
        for h in humans:
            h.update_position()
            people.people.append(h.get_person_data())

        peoplePub.publish(people)
        rospy.sleep(0.5)
