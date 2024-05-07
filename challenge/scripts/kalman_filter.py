#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

# Global variables
wl = 0.0
wr = 0.0
r = 40.0      # noise covariance
h = 1.0       # measurement map scalar
q = 10.0      # initial estimates covariance
p = 0.0       # initial error covariance (must be zero)
est = 0.0     # initial estimated state
k = 0.0       # initial kalman gain 

def kalman(input):
    global r, h, q, p, est, k
    k = (p*h)/( h * p * h + r)      # update kalman gain
    est = est + k * (input - h * est)  # update estimation
    p = (1 - k * h) * p + q    # update error covariance
    return est

# Getting Wl
def callback_wl(msg):
    global wl
    wl = msg.data

# Getting Wr
def callback_wr(msg):
    global wr
    wr = msg.data

# Main loop
if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("kalman")
    print("Kalman is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Subscribers
    wr_sub = rospy.Subscriber("wr", Float32, callback_wr)
    wl_sub = rospy.Subscriber("wl", Float32, callback_wl)

    # Publishers
    wr_pub = rospy.Publisher("kf_wr", Float32 , queue_size=10)
    wl_pub = rospy.Publisher("kf_wl", Float32 , queue_size=10)

    try:

        while not rospy.is_shutdown():

            # Aply kalman filter
            kf_wr = kalman(wr)
            kf_wl = kalman(wl)

            # Publish filtered signal
            wr_pub.publish(kf_wr)
            wl_pub.publish(kf_wl)
            
            #Wait and repeat
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node