#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#Global variables
msgRobot = Twist()
# pose = JointState()

#Wheel constants
R = 0.05
L = 0.191

# Obtaining cmd vel
def callback_cmd(msg):
    global msgRobot
    msgRobot.linear.x = msg.linear.x
    msgRobot.angular.z = msg.angular.z


# Getting Position - Solver
def pose_obtainal(lin_vel, ang_vel):
    ang = ang_vel * np.sin(ang_vel)
    x_vel = lin_vel * np.cos(ang)
    y_vel = lin_vel * np.sin(ang)

    return x_vel,y_vel
   
# Differential Model Solver
def diff_model(lin_vel, ang_vel):
    # One wheel is twice as fast as other wheel
    # wr = 2*wl
    # wl = 1/2 wr
    # wr = (ang*L) / (3*R)
    # wl = (2*ang*L) / (3*R)

    # wr = (2*lin_vel) / (R)
    # wl = - (2*lin_vel) / (R) 
    # wl = (2*lin_vel) / (R) - 2*wr

    wl = ((2 * lin_vel) - (L * ang_vel)) / (2 * R)
    wr = ((2 * lin_vel) / R) - wl

    return wr, wl
   
if __name__=="__main__":
    rospy.init_node("puzzlebot_kinematic_model")

    # Publishers
    # pose_pub = rospy.Publisher("pose", Float32 , queue_size=10)
    wr_pub = rospy.Publisher("wr", Float32 , queue_size=10)
    wl_pub = rospy.Publisher("wl", Float32 , queue_size=10)
    
    # Subscribers
    rospy.Subscriber("cmd_vel", Twist , callback=callback_cmd)

    pose_pub = rospy.Publisher("pose", Twist , queue_size=10)

    rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Main loop
    try:
        while not rospy.is_shutdown():
            
            # cmd_sub = rospy.Subscriber("wr", Twist, msgRobot)
            # rospy.spin() #last line without while

            linear_vel = msgRobot.linear.x
            angular_vel = msgRobot.angular.z

            # print("Values gotten linear: ", linear_vel)
            # print("Values gotten angular: ", angular_vel)

            wr,wl = diff_model(linear_vel, angular_vel)

            print("Values of right wheel: ", wr)
            print("Values of left wheel: ", wl)

            wr_pub.publish(wr)
            wl_pub.publish(wl)
            
            # x_dot, y_dot = pose_obtainal(linear_vel,angular_vel)
            # msgRobot.linear.x = x_dot
            # msgRobot.linear.y = y_dot
            # msgRobot.angular.z = angular_vel
            # header.stamp = rospy.Time.now()
            # pose.name = ["pose"]
            # pose.positionX = [x]
            # pose.positionY = [y]

            # pose_sub.publish(pose)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
