#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


#Global variables
msgRobot = Twist()

#Wheel constants
R = 5
WHEELBASE = 19
L = 16

# Obtaining cmd vel
def callback_cmd(msg):
    global msgRobot
    msgRobot.linear.x = msg.linear.x
    msgRobot.angular.z = msg.angular.z
   
# Differential Model Solver
def diff_model(lin_vel, ang_vel):
    wl = ((2 * lin_vel) - (L * ang_vel)) / (2 * R)
    wr = ((2 * lin_vel) / R) - wl

    return wr, wl
   
if __name__=="__main__":
    rospy.init_node("puzzlebot_kinematic_model")

    # Publishers
    wr_pub = rospy.Publisher("wr", Float32 , queue_size=10)
    wl_pub = rospy.Publisher("wl", Float32 , queue_size=10)
    
    # Subscribers
    rospy.Subscriber("cmd_vel", Twist , callback=callback_cmd)

    rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Main loop
    try:
        while not rospy.is_shutdown():
            linear_vel = msgRobot.linear.x
            angular_vel = msgRobot.angular.z

            wr,wl = diff_model(linear_vel, angular_vel)

            print("Values of right wheel: ", wr)
            print("Values of left wheel: ", wl)

            wr_pub.publish(wr)
            wl_pub.publish(wl)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
