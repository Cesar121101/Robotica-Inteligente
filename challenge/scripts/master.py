#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
from tf.transformations import euler_from_quaternion # imported in controlley.py

# Global Variables
robot_odom = Odometry()
aurco_pose = Pose()
aruco_id = 0
robot_state = 1
robot_position = 0.0
robot_orientation = 0.0
points_msg = Float64MultiArray()
state_flag_msg = True               # State flag = True == finished process
state_flag = False                  # If state_flag == false. Process has not finished
command = Twist()

def callback_robot_odom(msg):
    global poseRobot
    poseRobot = msg.pose.pose

def callback_robot_position(msg):
    global robot_position_msg
    robot_position_msg = msg.data

def callback_robot_orientation(msg):
    global robot_orientation_msg
    robot_orientation_msg = msg.data

def callback_aruco_id(msg):
    global aruco_id
    aruco_id = msg.data

def callback_aruco_pose(msg):
    global aruco_pose
    aruco_pose = msg.data

def callback_state_flag(msg):
    global state_flag_msg
    state_flag_msg = msg.data

if __name__=='__main__':
    #* Initialize and Setup node
    rospy.init_node("master")
    print("MASTER: Global Staetes of Puzzlebot")

    #* Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #* Subscribers
    rospy.Subscriber("/odom", Float32, callback_robot_odom)                         #? If we use Odometry topic
    rospy.Subscriber("/controller/orientReal", Float32, callback_robot_orientation) #? use controller published odometry
    rospy.Subscriber("/aruco_id", int, callback_aruco_id)
    rospy.Subscriber("/aruco_pose", Pose, callback_aruco_pose)
    rospy.Subscriber("/state_flag", bool, callback_state_flag)
    
    #* Publishers
    points_pub = rospy.Publisher("/points", Float64MultiArray, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    robot_state_pub = rospy.Publisher("/state", int, queue_size=10)
    # ROBOT STATES
    # 0 = avoid ostacle
    # 1 = search for aruco
    # 2 = turn puzzlebot to search for aruco
    # 3 = go to aruco
    # 4 = grab crate
    # 5 = go to unloading spot
    # 6 = leave crate

    # Initialize local variables
    points_data = []

    try:
        while not rospy.is_shutdown():

            # #? If we use Odometry topic
            robot_position = poseRobot
            (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])

            #? Use controller published odometry
            robot_orientation = robot_orientation_msg

            state_flag = state_flag_msg
            
            # TODO: 0. Is there obstacle - LiDAR sensor
            # * Controller does this automatically
            # if(lidar_sensor):
            #     turn_to_avoid_obstacle()
            # else:

            #     ROBOT STATES
            # TODO: 1. Search for aruco
            if(robot_state == 1):
                aruco_position = aruco_pose
                if(state_flag):
                    if(len(aruco_position) > 0):        # check if an aruco was detected
                        #* Change state
                        robot_state = 2                 # when aruco was detected
                    else:
                        print("No ARUCO detected.")
                else:                               # when aruco was not found
                    # Holiiiiiiiiiiiii
                    robot_orientation = robot_orientation + (np.pi)/4                                               # turns 45 deg
                    robot_point_data = [robot_position.position.x, robot_position.position.y, robot_orientation]    # point to turn robot to the right
                    robot_point_data = [float(robot_point_data[j]) for j in range(len(robot_point_data))]           # make sure all data is float
                    points_msg.data = robot_point_data                                                              # sets it to just one point,  rewrites the array
                    points_pub.Publish(points_msg)
                    #! CHANGE state_flag in aruco.py
                    
            #TODO: 2. go_to_aruco
            elif(robot_state == 2):
                if(state_flag):
                    robot_state = 3
                else:
                    aruco_point_data = [aruco_position.position.x, aruco_position.position.y, aruco_position.position.z]        # calculate to turn robot right
                    aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]                       # make sure all data is float
                    points_msg.data = aruco_point_data                                                                          # sets it to just one point,  rewrites the array
                    points_pub.Publish(points_msg)
                    # rospy.sleep(500)                                                                                            #! Wait to make sure ROS controller gets aruco
                    #! CHANGE state_flag in controller.py


            #TODO: 3. grab_aruco
            elif(robot_state == 3):
                #! MISSING code for gripper process
                if(state_flag):
                    robot_state = 5
                else:
                    print("Grabbing ARUCO.")

            # #TODO: 4. get away form aruco's base
            # elif(robot_state == 4):
            #     if(state_flag):
            #         robot_state = 5
            #     else:
            #         print("Moving away from ARUCO's base.")
            #         # TODO: Make Puzzlebot move backwards
            #         command.linear.x = 0.0
            #         command.linear.y = 0.0
            #         command.linear.z = 0.0
            #         command.angular.x = 0.0
            #         command.angular.y = 0.0
            #         command.angular.z = 0.0
            #         cmd_vel_pub.Publish(command)
            #         rospy.sleep(1000)
            #         robot_state = 5

            #TODO: 5. move towards goal
            elif(robot_state == 5):
                if(state_flag):
                    robot_state = 6
                else:
                    print("Moving towards unloading spot.")
                    #! CHANGE state_flag in controller.py

            #TODO: 6. leave crate
            elif(robot_state == 6):
                if(state_flag):
                    print("ALL PROCESS FINISHED!!")
                else:
                    #! MISSING gripper unload
                    print("Leaving ARUCO.")

            
            #TODO: broken state, turn off states
            else:
                print("Error in state logic... setting point topics empty and state to 0")
                robot_state = 0
                point_data = []
                points_msg.data = point_data
                points_pub.Publish(points_msg)

               
            robot_state_pub.publish(robot_state)

            rospy.spin()


            

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node