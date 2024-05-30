#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int16, Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import time
import numpy as np
from tf.transformations import euler_from_quaternion # imported in controlley.py

# Global Variables
robot_odom = Odometry()
aruco_pose = Pose()
aruco_id = -1
robot_state = 1
poseRobot = Pose()
points_msg = Float64MultiArray()
state_flag_msg = True               # State flag = True == finished process
state_flag = 0                  # If state_flag == 0. Process has not finished
servo_flag = 0
state5_flag = 0  
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
    aruco_pose = msg

def callback_state_flag(msg):
    global state_flag
    state_flag = msg.data

def callback_servo_flag(msg):
    global servo_flag
    servo_flag = msg.data

def callback_state5_flag(msg):
    global state5_flag
    state5_flag = msg.data

def center_aruco_position(aruco_pose):
    global command
    print("x: ", aruco_pose.position.x, "y: ", aruco_pose.position.y, "z: ", aruco_pose.position.y)
    
    if (aruco_pose.position.x > -0.1 and aruco_pose.position.x < 0.085):
        print("moving forward")
        command.linear.x = 0.03 #0.03
        command.angular.z = 0.0
    elif (aruco_pose.position.x < -0.1):
        print("turning right")
        command.linear.x = 0.0
        command.angular.z = 0.03
    elif(aruco_pose.position.x > 0.065):
        print("print left")
        command.linear.x = 0.0
        command.angular.z = -0.03



if __name__=='__main__':
    #* Initialize and Setup node
    rospy.init_node("master")
    print("MASTER: Global Staetes of Puzzlebot")

    #* Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #* Subscribers
    rospy.Subscriber("/odom", Odometry, callback_robot_odom)
    rospy.Subscriber("/aruco_id", Int16, callback_aruco_id)
    rospy.Subscriber("/aruco_pose", Pose, callback_aruco_pose)
    rospy.Subscriber("/state_flag", Int16, callback_state_flag)
    rospy.Subscriber("/servo_flag", Int16, callback_servo_flag)
    rospy.Subscriber("/state5_flag", Int16, callback_state5_flag)
    
    #* Publishers
    points_pub = rospy.Publisher("/points", Float64MultiArray, queue_size=10)
    robot_state_pub = rospy.Publisher("/state", Int16, queue_size=10)
    robot_state_flag = rospy.Publisher("/state_flag", Int16, queue_size=10)
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
            print("---------ENTERING---------------")

            # #? If we use Odometry topic
            robot_position = poseRobot
            (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])

            #*     ROBOT STATES
            #TODO: 8. leave crate
            if(robot_state == 8):
                print("JOB DONE!")
            
            #TODO: 7. Go to goal aruco
            elif(robot_state == 7):
                print("STATE 7: LEAVE CRATE")
                if(state_flag == 1):  # experimental data of optimal distance for the claw to grab the crate
                    robot_state = 8
                    print("Leave aruco")
                else:                    
                    aruco_point_data = [(robot_position.position.x + aruco_position.position.z*0.2*np.cos(robot_orientation)), (robot_position.position.y + aruco_position.position.z*0.2*np.sin(robot_orientation)), 0.0]        # calculate to turn robot right
                    aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]                       # make sure all data is float
                    
                    print("-------POSITION OF THE ROBOT---------")
                    print([robot_position.position.x, robot_position.position.y, robot_orientation])
                    print("-------POSITION OF THE ARUCO---------")
                    print(aruco_point_data)
                    points_msg.data = aruco_point_data          

            #TODO: 6. search aruco goal
            elif(robot_state == 6):
                print("STATE 6: SEARCHING GOAL")
                aruco_position = aruco_pose
                if(aruco_id == 1):        # search an specific arco
                    #* Change state
                    robot_state = 7                 # when aruco was detected
                    print("Aruco ID: ", aruco_id)
                else:                               # when aruco was not found
                    print("No ARUCO detected. Publishing cmd")

            #TODO: 5. move towards goal
            elif(robot_state == 5):
                print("STATE 5: MOVE TOWARDS GOAL")
                if(state5_flag == 1):
                    robot_state = 6
                    points_msg.data = []
                    print("Move to state 6")
                else:
                    print("Moving towards unloading spot.")
                    #! CHANGE state_flag in controller.py
            
            #TODO: 4. get away form aruco's base
            elif(robot_state == 4):
                if(state_flag == 1):
                    robot_state = 5
                    points_msg.data = [2.3, -0.9, 0.0]
                else:
                    print("Moving away from ARUCO's base.")
            
            #TODO: 3. grab_aruco
            elif(robot_state == 3):
                print("STATE 3: GRAB ARUCO")
                #! MISSING code for gripper process
                if(servo_flag == 1):
                    robot_state = 4
                else:
                    print("Grabbing ARUCO.")

            #TODO: 2. go_to_aruco
            elif(robot_state == 2):
                print("STATE 2: GO TO ARUCO")
                if(state_flag == 1):  # experimental data of optimal distance for the claw to grab the crate
                    robot_state = 3
                    #! Check if bug2 does when it finishes in current_state == 8
                else:                    
                    aruco_point_data = [(robot_position.position.x + aruco_position.position.z*0.67*np.cos(robot_orientation)), (robot_position.position.y + aruco_position.position.z*0.67*np.sin(robot_orientation)), 0.0]        # calculate to turn robot right
                    aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]                       # make sure all data is float
                    
                    print("-------POSITION OF THE ROBOT---------")
                    print([robot_position.position.x, robot_position.position.y, robot_orientation])
                    print("-------POSITION OF THE ARUCO---------")
                    print(aruco_point_data)
                    points_msg.data = aruco_point_data                                                                          # sets it to just one point,  rewrites the array

            # TODO: 1. Search for aruco
            elif(robot_state == 1):
                print("STATE 0: Search ARUCO")
                aruco_position = aruco_pose
                if(aruco_id != -1):        # check if an aruco was detected
                    #* Change state
                    robot_state = 2                 # when aruco was detected
                    print("Aruco ID: ", aruco_id)
                else:                               # when aruco was not found
                    print("No ARUCO detected. Publishing cmd")

            # TODO: 0. Is there obstacle - LiDAR sensor
            # * bug2.py -> Controller does this automatically
            
            #TODO: broken state, turn off states
            else:
                print("Error in state logic... setting point topics empty and state to 0")
                # robot_state = 0
                point_data = []
                points_msg.data = point_data

            print("  -------COMMANDS--------  ")
            print("STATE: ", robot_state)
            print("      ---POINTS---         ")
            print(points_msg)
            print("  -----------------------  ")
            print("---------------------------")

            points_pub.publish(points_msg)
            robot_state_pub.publish(robot_state)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node