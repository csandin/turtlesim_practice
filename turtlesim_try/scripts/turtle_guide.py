#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import *
from std_srvs.srv import Empty
from math import pi

def turtle_spawn():
    """ Spawn new turtles in desired locations """
    # Reset turtlesim window. Kill original turtle1 from startup
    rospy.wait_for_service('reset')
    reset = rospy.ServiceProxy('reset', Empty)
    reset()

    kill = rospy.ServiceProxy('kill', Kill)
    kill("turtle1")

    # Spawn new turtle1 in desired location
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(10.5, 5.5, pi/2, 'turtle1')

def guide_turtle():
    """ Begin guide turtle movement """
    # Initialize node
    rospy.init_node('turtle_guide')

    # Create velocity publisher
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Initialize vel_msg values
    vel_msg.linear.y, vel_msg.linear.z = 0, 0
    vel_msg.angular.x, vel_msg.angular.y, = 0, 0

    vel_msg.linear.x = 2*pi
    vel_msg.angular.z = (2*pi)/5

    # Create publishing rate
    rate = rospy.Rate(10)

    # Movement loop
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        turtle_spawn()
        guide_turtle()
    except rospy.ROSInterruptException:
        pass
