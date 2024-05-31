#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16
import time

robot_state_msg = Int16()
robot_state_flag = Int16()

def callback_robot_state(msg):
    global robot_state_msg
    robot_state_msg = msg.data

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("servo")
    print("Servo is running")

    degrees = -60.0
    state_flag = 0.0

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Setup de publishers
    servo_pub = rospy.Publisher("servo", Float32, queue_size=10)
    robot_state_flag_pub = rospy.Publisher("/servo_flag", Int16, queue_size=10)
    rospy.Subscriber("/state", Int16, callback_robot_state)

    try:
        while not rospy.is_shutdown():
            if robot_state_msg == 3:
                if degrees < 70.0:
                    degrees += 5
            if robot_state_msg == 8:
                if degrees >= 0:
                    degrees -= 5

            if degrees == 65.0 and robot_state_msg == 3:
                state_flag = 1.0
            elif degrees == 5.0 and robot_state_msg == 8:
                state_flag = 1.0
            else:
                state_flag = 0.0

            time.sleep(0.1)
            servo_pub.publish(degrees)
            robot_state_flag_pub.publish(state_flag)

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node