#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose,Quaternion
from tf.transformations import quaternion_from_euler
import random

# Global variables
odom_result = Odometry()
pose_result = Pose()
twist_result = Twist()
lsr_dists = []
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
kr = 0.0
kl = 0.0
covariance = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
covariance_pred = np.matrix([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
angle_increment = 0.0
z_lidar = np.matrix([[0.0],
                    [0.0]])
corner_to_use = 0
mew = np.matrix([[0.0],
                 [0.0],
                 [0.0]])

def new_cal_r():
    return np.matrix([[0.1, 0.0],
                      [0.0, 0.02]])

# Calculate Q for k moment (current)
def cal_q(wr, wl, deltaTime, prevAngle):
    triang = np.matrix([[0.5*r*deltaTime*np.cos(prevAngle), 0.5*r*deltaTime*np.cos(prevAngle)],
                        [0.5*r*deltaTime*np.sin(prevAngle), 0.5*r*deltaTime*np.sin(prevAngle)],
                        [0.5*r*deltaTime*(2.0/l), 0.5*r*deltaTime*(2.0/l)]])
    s = np.matrix([[kr*np.abs(wr), 0.0],[0.0, kl*np.abs(wl)]])
    q = triang*s*np.transpose(triang)
    return q

# Calculate H
def cal_h(Angle, linearV, deltaTime):
    return np.matrix([[1.0, 0.0, -deltaTime*linearV*np.sin(Angle)],
                      [0.0, 1.0, deltaTime*linearV*np.cos(Angle)],
                      [0.0, 0.0, 1.0]])
    
# Calculate G for k moment (current)
def cal_g_z(m,x_robot, y_robot, theta_robot):
    #print("m",m)
    delta_x = m.item(0,0) - x_robot 
    delta_y = m.item(1,0) - y_robot
    p = np.power(delta_x,2) + np.power(delta_y,2)

    z = np.matrix([[np.sqrt(p)],
                   [np.arctan2(delta_y,delta_x)-theta_robot]])
    #print("z",z)

    g = np.matrix([[-1.0*delta_x/np.sqrt(p), -1.0*delta_y/np.sqrt(p), 0.0],
                   [delta_y/p, -1.0*delta_x/p, -1.0]])

    return g, z

# Calculate R IDK
def cal_r():
    return np.matrix([[0.0, 0.0],
                      [0.0, 0.0]]) #0.001

# Getting Wl
def callback_wl(msg):
    global wl
    wl = msg.data

# Getting Wr
def callback_wr(msg):
    global wr
    wr = msg.data

def callback_lsr(msg):
    global lsr_dists, angle_increment
    angle_increment = msg.angle_increment
    rang = msg.ranges
    new = []
    for x in range(0, len(rang)):
        new.append(rang[x])
        if rang[x] > 10:
            new[x] = 10
        elif rang[x] < -10:
            new[x] = -10
    lsr_dists = new
    

        
        
def gaussian_filter_1d(data, sigma=1):
    # Calculate the size of the filter window
    size = int(6 * sigma + 1)
    if size % 2 == 0:  # Ensure size is odd
        size += 1
    
    # Create the 1D Gaussian kernel
    kernel = np.exp(-(np.arange(size) - size // 2) ** 2 / (2 * sigma ** 2))
    kernel /= np.sum(kernel)  # Normalize the kernel to sum to 1
    
    # Apply the filter to the data
    filtered_data = np.convolve(data, kernel, mode='same')
    
    return filtered_data

def lowpass_filter_1d(data, window_size):
    # Define the kernel for the moving average
    kernel = np.ones(window_size) / window_size
    
    # Apply the filter to the data
    filtered_data = np.convolve(data, kernel, mode='same')
    
    return filtered_data

def find_corners(ranges, m, orientation, x_robot, y_robot):
    print("Orientiation", orientation)
    print("XR:",x_robot,"YR:",y_robot)
    #ranges = lowpass_filter_1d(ranges, 5)
    #ranges = gaussian_filter_1d(ranges, 1.5)
    corners = []
    dy = [ranges[i+1] - ranges[i] for i in range(len(ranges)-1)]
    #print(dy)
    for x in range(2, len(dy)-2):
        if dy[x-1] > 0.005 and dy[x+1] < -0.005 and dy[x-2] > 0.0055 and dy[x+2] < -0.0055:
            x_point = (ranges[x]*np.cos((x*0.31386)*(np.pi/180)+orientation)) + x_robot
            y_point = (ranges[x]*np.sin((x*0.31386)*(np.pi/180)+orientation)) + y_robot

            if np.sqrt((m.item(0,0) - x_point)*(m.item(0,0) - x_point)+(m.item(1,0)- y_point)*(m.item(1,0)- y_point)) < 1.0:
                corners.append(x)
        
    #for x in corners:
    #    x_point = ranges[x]*np.cos((x*0.31386)*(np.pi/180)+orientation) + x_robot
    #    y_point = ranges[x]*np.sin((x*0.31386)*(np.pi/180)+orientation) + y_robot

    #    corners_x.publish(x_point)
    #    corners_y.publish(y_point)
    return corners
        
def points_pubs(ranges, orientation, x_robot, y_robot):
    for x in range(0, len(ranges)):
        if ranges[x] > -10 and ranges[x] < 10:
            x_point = ranges[x]*np.cos((x*0.31386)*(np.pi/180) + orientation) + x_robot
            y_point = ranges[x]*np.sin((x*0.31386)*(np.pi/180) + orientation) + y_robot

            points_pub_x.publish(x_point)
            points_pub_y.publish(y_point)

# Main loop
if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("localisation")
    print("Localisation is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Subscribers
    wr_sub = rospy.Subscriber("/wr", Float32, callback_wr)
    wl_sub = rospy.Subscriber("/wl", Float32, callback_wl)
    lsr_sub = rospy.Subscriber("/scan", LaserScan, callback_lsr)

    # Publishers
    odom_pub = rospy.Publisher("odom", Odometry , queue_size=10)
    points_pub_x = rospy.Publisher("pointsx", Float32 , queue_size=10)
    points_pub_y = rospy.Publisher("pointsy", Float32 , queue_size=10)
    corners_x = rospy.Publisher("cornersx", Float32 , queue_size=10)
    corners_y = rospy.Publisher("cornersy", Float32 , queue_size=10)

    prevTime = rospy.get_time()
    try:

        while not rospy.is_shutdown():
            if len(lsr_dists) > 0:
                m_point = np.matrix([[3.34],
                                     [-1.87]])
                currentTime = rospy.get_time()
                deltaTime = currentTime-prevTime

                # Linear Speed and Angular Speed
                v = (r*((wr+wl)/2))
                w = (r*((wr-wl)/l))

                # Deltasmew.item(2,0)
                deltaDist = v*deltaTime
                deltaAngle = w*deltaTime
                # Currents
                xCurrent = xPrev + deltaDist*np.cos(prevAngle)
                yCurrent = yPrev + deltaDist*np.sin(prevAngle)
                zAngle = prevAngle + deltaAngle

                mew_pred = np.matrix([[xCurrent],
                                    [yCurrent],
                                    [zAngle]])
                # points_pubs(lsr_dists, zAngle,xCurrent,yCurrent)

                # Calculate Covariance
                Q = cal_q(wr, wl, deltaTime, prevAngle)
                H = cal_h(mew_pred.item(2,0), v, deltaTime)

                covariance_pred = H * covariance * np.transpose(H) + Q

                R = cal_r()

                corners = find_corners(lsr_dists, m_point,zAngle, xCurrent, yCurrent)
                if not(corners == []):
                    corner_to_use = corners[0]
                
                z_lidar = np.matrix([[lsr_dists[corner_to_use]],
                                    [(corner_to_use)*(np.pi/180.0)]])

                G, z_prediction = cal_g_z(m_point, xCurrent,yCurrent,zAngle)
                #print(G.shape, covariance.shape, np.transpose(G).shape, R.shape)
                Z = G * covariance_pred * np.transpose(G) + R

                K = covariance_pred * np.transpose(G) * np.linalg.pinv(Z)
                print("K", K)
                print("G:", G)
                print("COV:",covariance_pred)
                print("ZL:",z_lidar)
                print("ZP:",z_prediction)
                print("DeltaZ", z_lidar-z_prediction)
                mew = mew_pred + K * (z_lidar-z_prediction)

                I = np.matrix([[1.0,  0.0,  0.0],
                               [0.0,  1.0,  0.0], 
                               [0.0,  0.0,  1.0]])

                covariance = (I-(K * G)) * covariance_pred

                #print("X: ", mew.item(0 ,0),", Y: ",  mew.item(1, 0), ", Angle: ", mew.item(2, 0))

                # Update prevs
                xPrev = mew.item(0,0)
                yPrev = mew.item(1,0)
                prevAngle = mew.item(2,0)
                prevTime = currentTime

                # Build Pose
                pose_result.position.x = xPrev
                pose_result.position.y = yPrev
                pose_result.position.z = 0.0
                #       Build quaternion
                q_new = quaternion_from_euler(0.0, 0.0, prevAngle)
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
                #print(odom_result)

                # Publish
                odom_pub.publish(odom_result)

                #Wait and repeat
                loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialize and Setup node