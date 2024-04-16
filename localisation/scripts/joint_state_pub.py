#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform,Quaternion,TransformStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler


#Declare Variables to be used
jointstate_result = JointState()
tf_result = TFMessage()
transformS_current = TransformStamped()
transformSWL_current = TransformStamped()
transformSWR_current = TransformStamped()
transform_current = Transform()
transformWL_current = Transform()
transformWR_current = Transform()

odom_current = Odometry()
wl = 0.0
wr = 0.0
wl_pos = 0.0
wr_pos = 0.0

# Define the callback functions
def callback_wl(msg):
    global wl
    wl = msg.data

def callback_wr(msg):
    global wr
    wr = msg.data

#Define the callback functions
def callback_odom(msg):
    global odom_current
    odom_current = msg


if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("joint_state_pub")
    print("Joint State Publisher is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    odom_sub = rospy.Subscriber("odom", Odometry, callback_odom)
    wr_sub = rospy.Subscriber("wr", Float32, callback_wr)
    wl_sub = rospy.Subscriber("wl", Float32, callback_wl)

    #Setup de publishers
    tf_pub = rospy.Publisher("tf", TFMessage, queue_size=1)
    joints_pub = rospy.Publisher("joint_states", JointState , queue_size=10)

    # Starting values
    odom_current.pose.pose.position.x = 0.0
    odom_current.pose.pose.position.y = 0.0
    odom_current.pose.pose.position.z = 0.0
    odom_current.pose.pose.orientation.x = 0.0
    odom_current.pose.pose.orientation.y = 0.0
    odom_current.pose.pose.orientation.z = 0.0
    odom_current.pose.pose.orientation.w = 1.0
    transform_current.translation.x = 0.0
    transform_current.translation.y = 0.0
    transform_current.translation.z = 0.0
    transform_current.rotation.x = 0.0
    transform_current.rotation.y = 0.0
    transform_current.rotation.z = 0.0
    transform_current.rotation.w = 1.0
    transformS_current.child_frame_id ="base_link"
    transformS_current.header.frame_id ="odom"

    jointstate_result.name = ["rightWheel","leftWheel"]
    jointstate_result.position = [0.0, 0.0]

    # Wheel R
    transformSWR_current.child_frame_id ="wr_link"
    transformSWR_current.header.frame_id ="chassis"
    transformWR_current.translation.x = 0.05
    transformWR_current.translation.y = -0.0955
    transformWR_current.translation.z = 0.0
    transformWR_current.rotation.x = 0.0
    transformWR_current.rotation.y = 0.0
    transformWR_current.rotation.z = 0.0
    transformWR_current.rotation.w = 1.0

    # Wheel L
    transformSWL_current.child_frame_id ="wl_link"
    transformSWL_current.header.frame_id ="chassis"
    transformWL_current.translation.x = 0.05
    transformWL_current.translation.y = 0.0955
    transformWL_current.translation.z = 0.0
    transformWL_current.rotation.x = 0.0
    transformWL_current.rotation.y = 0.0
    transformWL_current.rotation.z = 0.0
    transformWL_current.rotation.w = 1.0

    prevTime = rospy.get_time()
    try:
        while not rospy.is_shutdown():
            currentTime = rospy.get_time()
            deltaTime = currentTime-prevTime

            # Joint states
            wr_pos = wr_pos + wr*deltaTime
            wl_pos = wl_pos + wl*deltaTime
            
            jointstate_result.position = [wr_pos, wl_pos]
            jointstate_result.header.stamp = rospy.Time.now() 

            qwl_new = quaternion_from_euler(0.0, wl_pos, 0.0)
            qwr_new = quaternion_from_euler(0.0, wr_pos, 0.0)

            transformWL_current.rotation.x = qwl_new[0]
            transformWL_current.rotation.y = qwl_new[1]
            transformWL_current.rotation.z = qwl_new[2]
            transformWL_current.rotation.w = qwl_new[3]

            transformWR_current.rotation.x = qwr_new[0]
            transformWR_current.rotation.y = qwr_new[1]
            transformWR_current.rotation.z = qwr_new[2]
            transformWR_current.rotation.w = qwr_new[3]

            # Moving Puzzlebot
            transform_current.translation.x = odom_current.pose.pose.position.x
            transform_current.translation.y = odom_current.pose.pose.position.y
            transform_current.translation.z = odom_current.pose.pose.position.z
            transform_current.rotation.x = odom_current.pose.pose.orientation.x
            transform_current.rotation.y = odom_current.pose.pose.orientation.y
            transform_current.rotation.z = odom_current.pose.pose.orientation.z
            transform_current.rotation.w = odom_current.pose.pose.orientation.w

            transformSWL_current.header.stamp = rospy.Time.now() 
            transformSWR_current.header.stamp = rospy.Time.now()
            transformSWL_current.transform = transformWL_current
            transformSWR_current.transform = transformWR_current
            
            transformS_current.header.stamp = rospy.Time.now() 
            transformS_current.transform = transform_current

            tf_result.transforms = [transformS_current, transformSWL_current, transformSWR_current]

            tf_pub.publish(tf_result)

            joints_pub.publish(jointstate_result)
            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialize and Setup node