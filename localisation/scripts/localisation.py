#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose,Quaternion
from tf.transformations import quaternion_from_euler

#Declare Variables to be used
odom_result = Odometry()
pose_result = Pose()
twist_result = Twist()

wl = 0.0
wr = 0.0
deltaAngle = 0.0
xCurrent = 0.0
xPrev = 0.0
yCurrent = 0.0
yPrev = 0.0
zAngle = 0.0
prevAngle = 0.0
l = 0.191
r = 0.05


# Define the callback functions
def callback_wl(msg):
    global wl
    wl = msg.data

def callback_wr(msg):
    global wr
    wr = msg.data

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("localisation")
    print("Localisation is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    wr_sub = rospy.Subscriber("wr", Float32, callback_wr)
    wl_sub = rospy.Subscriber("wl", Float32, callback_wl)

    #Setup de publishers
    odom_pub = rospy.Publisher("odom", Odometry , queue_size=10)

    prevTime = rospy.get_time()
    try:

        while not rospy.is_shutdown():
            currentTime = rospy.get_time()
            deltaTime = currentTime-prevTime

            # Linear Speed and Angular Speed
            v = (r/2)*(wr+wl)
            angSpeed = (r/l)*(wr-wl)

            # Deltas
            deltaDist = v*deltaTime
            deltaAngle = angSpeed*deltaTime

            # Currents
            xCurrent = xPrev + deltaDist*np.cos(prevAngle)
            yCurrent = yPrev + deltaDist*np.sin(prevAngle)
            zAngle = prevAngle + deltaAngle

            print("X: ", xCurrent,", Y: ",  yCurrent, ", Angle: ", zAngle)

            # Update prevs
            xPrev = xCurrent
            yPrev = yCurrent
            prevAngle = zAngle
            prevTime = currentTime

            # Build Pose
            pose_result.position.x = xCurrent
            pose_result.position.y = yCurrent
            pose_result.position.z = 0.0
            #       Build quaternion
            q_new = quaternion_from_euler(0.0, 0.0, zAngle)
            pose_result.orientation.x = q_new[0]
            pose_result.orientation.y = q_new[1]
            pose_result.orientation.z = q_new[2]
            pose_result.orientation.w = q_new[3]

            # Build twist
            twist_result.linear.x = v
            twist_result.linear.y = 0.0
            twist_result.linear.z = 0.0
            twist_result.angular.z = angSpeed
            twist_result.angular.x = 0.0
            twist_result.angular.y = 0.0

            # Build odom
            odom_result.pose.pose = pose_result
            odom_result.twist.twist = twist_result
            print(odom_result)

            # Publish
            odom_pub.publish(odom_result)

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialize and Setup node