#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

#Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0
dt = 1/100.0
a = l/2
J = (4/3)*m*a*a

# Setup Variables to be used
result = JointState()

# Declare the input Message

# Declare the  process output message


#Define the callback functions
def callback_tau(msg):
    global Tau
    Tau = msg.data

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    tau_sub = rospy.Subscriber("tau", Float32, callback_tau)

    #Setup de publishers
    joints_pub = rospy.Publisher("joint_states", JointState , queue_size=10)

    print("The SLM sim is Running")
    try:
        
        while not rospy.is_shutdown():
            x1 += x2*dt
            x2_dot = (1/J)*(Tau-m*g*a*np.cos(x1)-k*x2)
            x2 += (x2_dot*dt)

            result.header.stamp = rospy.Time.now()
            result.name = ["joint2"]
            result.position = [x1]
            result.velocity = [x2]

            print("Tau:", Tau)
            print(x1, x2, x2_dot)
            print(dt)

            joints_pub.publish(result)

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node