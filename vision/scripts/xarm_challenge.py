#!/usr/bin/env python
import rospy
import numpy as np
from numpy.linalg import inv
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

qr = Pose()
newPose = Pose()

def callback(msg):
    global qr
    qr = msg.pose

if __name__ == '__main__':
    rospy.init_node("xarm_qr_controller")
    print("Running QR Code Pose Controller")
    rospy.Subscriber("/object_position", PoseStamped, callback)
    pub = rospy.Publisher("/qr_pose", Pose, queue_size=10)
    rate = rospy.Rate(100)
    Vx = -float(input("X: "))
    Vy = -float(input("Y: "))
    Vz = -float(input("Z: "))

    V = np.array([[Vx],
                  [Vy],
                  [Vz]])

    Translation = np.array([[1,0,0],
                            [0,0,-1],
                            [0,1,0]])
    
    inverseT = inv(Translation)

    while not rospy.is_shutdown():
        
        P = np.array([[qr.position.x],
                     [qr.position.y],
                     [qr.position.z]])
        
        newOne = np.dot(inverseT, P)

        newOne = np.subtract(newOne, V)

        newPose.position.x = newOne[0][0]
        newPose.position.y = newOne[1][0]
        newPose.position.z = newOne[2][0]
        
        print("Sending: \n")
        print("X: " + str(newPose.position.x))
        print("Y: " + str(newPose.position.y))
        print("Z: " + str(newPose.position.z))
        pub.publish(newPose)
        rate.sleep()
    