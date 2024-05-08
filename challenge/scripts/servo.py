#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("servo")
    print("Servo is running")

    degrees = 0.0
    isClosing = 1

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Setup de publishers
    servo_pub = rospy.Publisher("servo", Float32, queue_size=10)

    try:
        while not rospy.is_shutdown():

            if degrees < 250.0 and isClosing == 1:
                degrees += 5
            elif degrees >= 0 and isClosing == 0:
                degrees -= 5

            if degrees == 250.0:
                isClosing = 0
            elif degrees == 0.0:
                isClosing = 1

            time.sleep(0.5)
            servo_pub.publish(degrees)

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node