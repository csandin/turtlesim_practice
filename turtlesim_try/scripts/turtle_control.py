#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def command():
    """ Start publisher and subscribers. Publish velocity info"""

    rospy.init_node('MoveTurtle')   # Initialize control node. Name: MoveTurtle

    move = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Get input
    print("Input movement parameters:")
    speed = input("Speed: ")
    distance = input("Distance: ")
    isForward = input("Forward: ")  # Binary

    if isForward == 1:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y, vel_msg.linear.z = 0, 0
    vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = 0, 0, 0

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while (current_distance < distance):
            move.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)

        vel_msg.linear.x = 0
        move.publish(vel_msg)

if __name__ == "__main__":
    try:
        command()
    except rospy.ROSInterruptException: pass
