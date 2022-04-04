#!/usr/bin/env python3

from people_msgs.msg import Person

class BasicPerson():
    """
    A class used to encapsulate the the basic parameters of a Person (location,
    direction, name, etc.) as well as provide an interface to change its position
    whenever the update_position() method is called. For this to work, the user
    must provide a function that takes only one parameter: a pointer to a Person 
    class. This function ill be called every time the update_position() is called.
    """

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
        """
        Update the potition of the person by calling the user-provided movement_policy()
        function.
        """
        self.movement_policy(self.person)
    
    def set_movement_policy(self, movement_policy):
        """
        Set a new momement policy.
        
        Attributes
        ----------
        movement_policy : function pointer
            A function that takes a pointer to a Person. The user can modify
            the values of the person within the given function.
        """
        self.movement_policy = movement_policy

    def get_person_data(self):
        return self.person