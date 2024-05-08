#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

qr = Pose()

def callback(msg):
    global qr
    qr = msg.pose

if __name__ == '__main__':
    rospy.init_node("xarm_qr_controller")
    print("Running QR Code Pose Controller")
    rospy.Subscriber("/object_position", PoseStamped, callback)
    pub = rospy.Publisher("/qr_pose", Pose, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print("Sending: \n")
        print("X: " + str(qr.position.x))
        print("Y: " + str(qr.position.y))
        print("Z: " + str(qr.position.z))
        pub.publish(qr)
        rate.sleep()
    