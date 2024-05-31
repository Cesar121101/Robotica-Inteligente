#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
from tf.transformations import euler_from_quaternion

#* Global Variables
state_flag_msg = True
aruco_id = -1
robot_state = 1
state_flag = 0
servo_flag = 0
state5_flag = 0  
final_zone = -1
robot_odom = Odometry()
aruco_pose = Pose()
poseRobot = Pose()
points_msg = Float64MultiArray()
command = Twist()

def callback_robot_odom(msg):
    global poseRobot
    poseRobot = msg.pose.pose

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

def callback_final_zone(msg):
    global final_zone
    final_zone = msg.data
    
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
    rospy.Subscriber("/final_zone", Int16, callback_final_zone)
    
    #* Publishers
    points_pub = rospy.Publisher("/points", Float64MultiArray, queue_size=10)
    robot_state_pub = rospy.Publisher("/state", Int16, queue_size=10)
    robot_state_flag = rospy.Publisher("/state_flag", Int16, queue_size=10)

    #* Initialize local variables
    points_data = []

    try:
        while not rospy.is_shutdown():
            robot_position = poseRobot
            (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])

            #*     ROBOT STATES
            #TODO: 8. Leave box
            if(robot_state == 8):
                print("JOB DONE!")
            
            #TODO: 7. Go to goal aruco
            elif(robot_state == 7):
                print("STATE 7: LEAVE CRATE")
                if(state_flag == 1):
                    robot_state = 8
                    print("Leave aruco")
                else:                    
                    aruco_point_data = [(robot_position.position.x + aruco_position.position.z*0.3*np.cos(robot_orientation)), (robot_position.position.y + aruco_position.position.z*0.3*np.sin(robot_orientation)), 0.0]
                    aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]
                    
                    print("-------POSITION OF THE ROBOT---------")
                    print([robot_position.position.x, robot_position.position.y, robot_orientation])
                    print("-------POSITION OF THE ARUCO---------")
                    print(aruco_point_data)
                    points_msg.data = aruco_point_data          

            #TODO: 6. Search aruco goal
            elif(robot_state == 6):
                print("STATE 6: SEARCHING GOAL")
                aruco_position = aruco_pose
                if(aruco_id == final_zone):        # Search specific aruco
                    robot_state = 7
                    print("Aruco ID: ", aruco_id)
                else:
                    print("No ARUCO detected. Publishing cmd")

            #TODO: 5. Move towards goal
            elif(robot_state == 5):
                print("STATE 5: MOVE TOWARDS GOAL")
                if(state5_flag == 1):
                    robot_state = 6
                    points_msg.data = []
                    print("Move to state 6")
                else:
                    print("Moving towards unloading spot.")
            
            #TODO: 4. Get away form aruco's base
            elif(robot_state == 4):
                if(state_flag == 1):
                    robot_state = 5
                    points_msg.data = [2.35, -1.15, 0.0]
                else:
                    print("Moving away from ARUCO's base.")
            
            #TODO: 3. Grab_aruco
            elif(robot_state == 3):
                print("STATE 3: GRAB ARUCO")
                if(servo_flag == 1):
                    robot_state = 4
                else:
                    print("Grabbing ARUCO.")

            #TODO: 2. Go_to_aruco
            elif(robot_state == 2):
                print("STATE 2: GO TO ARUCO")
                if(state_flag == 1):
                    robot_state = 3
                else:                    
                    aruco_point_data = [(robot_position.position.x + aruco_position.position.z*0.67*np.cos(robot_orientation)), (robot_position.position.y + aruco_position.position.z*0.67*np.sin(robot_orientation)), 0.0]        # calculate to turn robot right
                    aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]
            
                    print("-------POSITION OF THE ROBOT---------")
                    print([robot_position.position.x, robot_position.position.y, robot_orientation])
                    print("-------POSITION OF THE ARUCO---------")
                    print(aruco_point_data)
                    points_msg.data = aruco_point_data                                                                       

            # TODO: 1. Search for aruco
            elif(robot_state == 1):
                print("STATE 0: Search ARUCO")
                aruco_position = aruco_pose
                if(aruco_id != -1):        # If Aruco is detected change state
                    robot_state = 2
                    print("Aruco ID: ", aruco_id)
                else:
                    print("No ARUCO detected. Publishing cmd")

            #TODO: broken state, turn off states
            else:
                print("Error in state logic... setting point topics empty and state to 0")
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
        pass 