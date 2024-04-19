#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

msgRobot = Twist()

if __name__=='__main__':

    while not rospy.is_shutdown():
        rospy.init_node("cmd_publisher")
        rate = rospy.Rate(100)
        cmd_vel = 0.0

        cmd_pub = rospy.Publisher("cmd_vel", Twist , queue_size=10)

        linear_vel = rospy.get_param("/linear_velocity", int())
        angular_vel = rospy.get_param("/angular_velocity", int())

        msgRobot.linear.x = linear_vel
        msgRobot.angular.z = angular_vel

        print("\nValues published: ")
        print(linear_vel)
        print(angular_vel)

        cmd_pub.publish(msgRobot)
        rate.sleep()

    # rate.sleep()