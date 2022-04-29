#!/usr/bin/env python3

import rospy
import rospkg

from BasicPerson import BasicPerson

from people_msgs.msg import People,Person

# Make a function that does not modify the position.
# It also does not modify the direction. As such the effect is
# a human that is not moving but appears to have a direction it
# is facing.
def stay_put(person):
    pass


if __name__ == "__main__":
    #get path of the weights from rospkg so we can use it relative
    rospack = rospkg.RosPack()

    rospy.init_node('people_sim')
    
    peoplePub = rospy.Publisher('people', People , queue_size=10)

    r = rospy.Rate(5) # 5hz

    # Declare all the humans that will be generated
    human1 = BasicPerson('bob', -2, 2, 0, 0.0, 0.0, 0, 0.0, stay_put)

    humans = [human1]

    people = People()    
    people.header.frame_id = 'map'

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        people.people = []
        for h in humans:
            h.update_position() # Call the update function for each human
            people.people.append(h.get_person_data()) # Append to the people list

        peoplePub.publish(people) # Publish human locations
        rospy.sleep(0.5)