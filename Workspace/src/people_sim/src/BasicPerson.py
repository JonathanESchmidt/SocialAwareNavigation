#!/usr/bin/env python3

from people_msgs.msg import Person

class BasicPerson():

    def __init__(self, name='', x=0, y=0, z=0, dx=0, dy=0, dz=0, reliability =0 , movement_policy = None):
        self.person = Person()
        self.person.name = name
        self.person.position.x = x
        self.person.position.y = y
        self.person.position.z = z
        self.person.velocity.x = dx
        self.person.velocity.y = dy
        self.person.velocity.z = dz
        self.person.reliability = reliability
        self.movement_policy = movement_policy

    def update_position(self):
        self.movement_policy(self.person)
    
    def set_movement_policy(self, movement_policy):
        self.movement_policy = movement_policy

    def get_person_data(self):
        return self.person