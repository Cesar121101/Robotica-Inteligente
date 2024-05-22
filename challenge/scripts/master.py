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
# robot_position = 0.0
# robot_orientation = 0.0
poseRobot = Pose()
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
    aruco_pose = msg

def callback_state_flag(msg):
    global state_flag_msg
    state_flag_msg = msg.data

def init_command():
    global command
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

def center_aruco_position(aruco_pose):
    global command


    x1 = 140  # x-coordinate of the top-left corner of the ROI
    x2 = 500  # Width of the ROI

    #Center of the puzzlebot
    puzzlebot_x = (x2-x1) /2

    print("------------TEST FOR FINDING CENTER OF CAMERA AND CENTER OR ARUCO-----------------")
    print("Center Image :" + str(puzzlebot_x))

    print("x: ", aruco_pose.position.x, "y: ", aruco_pose.position.y, "z: ", aruco_pose.position.y)
    
    if (aruco_pose.position.x > -0.1 and aruco_pose.position.x < 0.085):
    #if (aruco_pose.position.x > -0.1 and aruco_pose.position.x < 0.065):
        print("moving forward")
        command.linear.x = 0.03 #0.03
        command.angular.z = 0.0
    elif (aruco_pose.position.x < -0.1):
        command.linear.x = 0.0
        command.angular.z = 0.00
    elif(aruco_pose.position.x > 0.065):
        command.linear.x = 0.0
        command.angular.z = -0.01



if __name__=='__main__':
    #* Initialize and Setup node
    rospy.init_node("master")
    print("MASTER: Global Staetes of Puzzlebot")

    #* Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #* Subscribers
    rospy.Subscriber("/odom", Float32, callback_robot_odom)                         #? If we use Odometry topic
    # rospy.Subscriber("/controller/orientReal", Float32, callback_robot_orientation) #? use controller published odometry
    # rospy.Subscriber("/aruco_id", int, callback_aruco)
    rospy.Subscriber("/odom", Odometry, callback_robot_odom)
    rospy.Subscriber("/aruco_id", Int16, callback_aruco_id)
    rospy.Subscriber("/aruco_pose", Pose, callback_aruco_pose)
    rospy.Subscriber("/state_flag", Int16, callback_state_flag)
    
    #* Publishers
    points_pub = rospy.Publisher("/points", Float64MultiArray, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    robot_state_pub = rospy.Publisher("/state", Int16, queue_size=10)
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

    init_command()

    try:
        while not rospy.is_shutdown():
            print("---------ENTERING---------------")

            # #? If we use Odometry topic
            robot_position = poseRobot
            (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])

            # #? Use controller published odometry
            # robot_orientation = robot_orientation_msg

            state_flag = state_flag_msg

            #*     ROBOT STATES
            #TODO: 6. leave crate
            if(robot_state == 6):
                print("STATE 6: LEAVE CRATE")
                if(state_flag):
                    print("ALL PROCESS FINISHED!!")
                else:
                    #! MISSING gripper unload
                    print("Leaving ARUCO.")
            
            #TODO: 5. move towards goal
            elif(robot_state == 5):
                print("STATE 5: MOVE TOWARDS GOAL")
                if(state_flag):
                    robot_state = 6
                else:
                    print("Moving towards unloading spot.")
                    #! CHANGE state_flag in controller.py
            
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
            #         cmd_vel_pub.publish(command)
            #         rospy.sleep(1000)
            #         robot_state = 5
            
            #TODO: 3. grab_aruco
            elif(robot_state == 3):
                print("STATE 3: GRAB ARUCO")
                #! MISSING code for gripper process
                if(state_flag):
                    robot_state = 5
                else:
                    print("Grabbing ARUCO.")

            #TODO: 2. go_to_aruco
            elif(robot_state == 2):
                print("STATE 2: GO TO ARUCO")
                if(aruco_pose.position.z < 0.162):
                    robot_state = 3
                    command.linear.x = 0.0
                    command.angular.z = 0.0 
                else:
                    # aruco_point_data = [robot_position.position.x + aruco_position.position.x, robot_position.position.y + aruco_position.position.z, "N"]        # calculate to turn robot right
                    # aruco_point_data = [float(aruco_point_data[j]) for j in range(len(aruco_point_data))]                       # make sure all data is float
                    
                    # print("-------POSITION OF THE ROBOT---------")
                    # print([robot_position.position.x, robot_position.position.y, robot_orientation])
                    # print("-------POSITION OF THE ARUCO---------")
                    # print(aruco_point_data)
                    # points_msg.data = aruco_point_data                                                                          # sets it to just one point,  rewrites the array
                    # points_pub.publish(points_msg)
                    # rospy.sleep(500)                                                                                            #! Wait to make sure ROS controller gets aruco
                    #! CHANGE state_flag in controller.py
                    center_aruco_position(aruco_pose=aruco_pose)
            
            # TODO: 1. Search for aruco
            elif(robot_state == 1):
                print("STATE 0: Search ARUCO")
                aruco_position = aruco_pose
                print(aruco_id)
                if(state_flag):
                    if(aruco_id != -1):        # check if an aruco was detected
                        #* Change state
                        print("change state")
                        robot_state = 2                 # when aruco was detected
                        command.linear.x = 0.0
                        command.angular.z = 0.0
                    else:
                        print("No ARUCO detected. Pubkishing cmd")
                        command.linear.x = 0.0
                        command.angular.z = 0.2
                        
                else:                               # when aruco was not found
                    # Holiiiiiiiiiiiii
                    print("No aruco detected 2")
                    robot_orientation = robot_orientation + (np.pi)/4                                               # turns 45 deg
                    robot_point_data = [robot_position.position.x, robot_position.position.y, robot_orientation]    # point to turn robot to the right
                    robot_point_data = [float(robot_point_data[j]) for j in range(len(robot_point_data))]           # make sure all data is float
                    print("-------POSITION OF THE ROBOT---------")
                    print(robot_point_data)
                    points_msg.data = robot_point_data                                                              # sets it to just one point,  rewrites the array
                    # points_pub.publish(points_msg)
                    #! CHANGE state_flag in aruco.py

                    #? If better only publixhing cmd_vel
                    # command.linear.x = 0.0
                    # command.angular.z = 0.5
                    # cmd_vel_pub.publish(command)
                    # rospy.sleep(100)
                    # command.linear.x = 0.0
                    # command.angular.z = 0.0
                    # cmd_vel_pub.publish(command)

            
            # TODO: 0. Is there obstacle - LiDAR sensor
            # * Controller does this automatically
            # if(lidar_sensor):
            #     turn_to_avoid_obstacle()
            # else:
            
            #TODO: broken state, turn off states
            else:
                print("Error in state logic... setting point topics empty and state to 0")
                robot_state = 0
                point_data = []
                points_msg.data = point_data
                # points_pub.publish(points_msg)

               
            robot_state_pub.publish(robot_state)
            print("-------COMMAND---------")
            print(command)
            cmd_vel_pub.publish(command)
            points_pub.publish(points_msg)

            loop_rate.sleep()


            

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node