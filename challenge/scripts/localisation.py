#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose,Quaternion
from tf.transformations import quaternion_from_euler

# Global variables
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
l = 0.180
r = 0.05
kr = 1.0
kl = 1.0
covariance = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])

# Calculate Q for k moment (current)
def cal_q(wr, wl, deltaTime, prevAngle):
    triang = np.matrix([[0.5*r*deltaTime*np.cos(prevAngle), 0.5*r*deltaTime*np.cos(prevAngle)],[0.5*r*deltaTime*np.sin(prevAngle), 0.5*r*deltaTime*np.sin(prevAngle)],[0.5*r*deltaTime*(2.0/l),0.5*r*deltaTime*(2.0/l)]])
    s = np.matrix([[kr*np.abs(wr), 0.0],[0.0, kl*np.abs(wl)]])
    q = triang*s*np.transpose(triang)
    return q

# Calculate H
def cal_h(prevAngle, linearV, deltaTime):
    return np.matrix([[1.0, 0.0, -deltaTime*linearV*np.sin(prevAngle)],[0.0, 1.0, deltaTime*linearV*np.cos(prevAngle)],[0.0, 0.0, 1.0]])
    
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
    rospy.init_node("localisation")
    print("Localisation is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Subscribers
    wr_sub = rospy.Subscriber("wr", Float32, callback_wr)
    wl_sub = rospy.Subscriber("wl", Float32, callback_wl)

    # Publishers
    odom_pub = rospy.Publisher("odom", Odometry , queue_size=10)

    prevTime = rospy.get_time()
    try:

        while not rospy.is_shutdown():
            currentTime = rospy.get_time()
            deltaTime = currentTime-prevTime

            # Linear Speed and Angular Speed
            v = (r*((wr+wl)/2))
            w = (r*((wr-wl)/l))

            # Deltas
            deltaDist = v*deltaTime
            deltaAngle = w*deltaTime

            # Currents
            xCurrent = xPrev + deltaDist*np.cos(prevAngle)
            yCurrent = yPrev + deltaDist*np.sin(prevAngle)
            zAngle = prevAngle + deltaAngle

            print("X: ", xCurrent,", Y: ",  yCurrent, ", Angle: ", zAngle)

            # Calculate Covariance
            Q = cal_q(wr, wl, deltaTime, prevAngle)
            H = cal_h(prevAngle, v, deltaTime)

            covariance = H*covariance*np.transpose(H) + Q

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
            twist_result.angular.z = w
            twist_result.angular.x = 0.0
            twist_result.angular.y = 0.0

            # Build odom
            odom_result.header.stamp = rospy.Time.now()
            odom_result.header.frame_id = "odom"
            odom_result.child_frame_id = "base_link"
            odom_result.pose.pose = pose_result

            new_cov = [covariance.item(0, 0), covariance.item(0, 1), 0.0, 0.0, 0.0, covariance.item(0, 2), 
                       covariance.item(1, 0), covariance.item(1, 1), 0.0, 0.0, 0.0, covariance.item(1, 2),
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       covariance.item(2, 0), covariance.item(2, 1), 0.0, 0.0, 0.0, covariance.item(2, 2)]
            odom_result.pose.covariance = new_cov
            odom_result.twist.twist = twist_result
            print(odom_result)

            # Publish
            odom_pub.publish(odom_result)

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialize and Setup node