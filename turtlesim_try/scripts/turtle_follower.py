#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from math import pow, atan2, sqrt, pi

class FollowerTurtle:
    # Initialize
    def __init__(self):
        # Spawn new turtle2
        rospy.wait_for_service('spawn')
        spawn = rospy.ServiceProxy('spawn', Spawn)
        spawn(5.5, 5.5, 0.0, "turtle2")

        pen_color = rospy.ServiceProxy('/turtle2/set_pen', SetPen)
        pen_color(118, 224, 94, 1, False)

        # Create node
        rospy.init_node("turtle_follower")

        # Initialize publisher and subscriber
        self.pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.guidesub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.selfsub = rospy.Subscriber('/turtle2/pose', Pose, self.my_pose)

        self.vel_msg = Twist()
        self.guide_pose = Pose()
        self.my_pose = Pose()

        self.rate = rospy.Rate(10)

        self.vel_msg.linear.y, self.vel_msg.linear.z = 0, 0
        self.vel_msg.angular.x, self.vel_msg.angular.y = 0, 0

    def update_pose(self, data):
        """ Callback function to update guide turtle pose """
        self.guide_pose = data
        self.guide_pose.x = round(self.guide_pose.x, 4)
        self.guide_pose.y = round(self.guide_pose.y, 4)

    def my_pose(self, data):
        """ Callback function to update follower turtle pose """
        self.my_pose = data
        self.my_pose.x = round(self.my_pose.x, 4)
        self.my_pose.y = round(self.my_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.my_pose.x), 2) +
                    pow((goal_pose.y - self.my_pose.y), 2))

    def linear_vel(self, goal_pose, constant=3):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.my_pose.y, goal_pose.x - self.my_pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.my_pose.theta)

    def follow(self):
        """ Function to begin following """
        tolerance = 0.001

        while not rospy.is_shutdown():
            if self.euclidean_distance(self.guide_pose) >= tolerance:
                self.vel_msg.linear.x = self.linear_vel(self.guide_pose)
                self.vel_msg.angular.z = self.angular_vel(self.guide_pose)
            else:
                self.vel_msg.linear.x, self.vel_msg.angular.z = 0, 0

            self.pub.publish(self.vel_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        x = FollowerTurtle()
        x.follow()
    except rospy.ROSInterruptException:
        pass
