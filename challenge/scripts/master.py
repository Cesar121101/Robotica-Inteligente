import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import time

robot_odom = Odometry()
aurco_pose = Pose()
robot_state = 1

def callback_robot_odom(msg):
    global robot_odom
    robot_odom = msg.data

def callback_aruco_pose(msg):
    global aruco_pose
    aruco_pose = msg.data

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("master")
    print("MASTER: Global Staetes of Puzzlebot")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Subscribers
    odom_sub = rospy.Subscriber("/odom", Float32, callback_robot_odom)
    pose_sub = rospy.Subscriber("/aruco_pose", Pose, callback_aruco_pose)
    
    #Publishers

    # ROBOT STATES
        # 0 = avoid ostacle
        # 1 = search of aruco
        # 2 = go to aruco
        # 3 = grab box
    # global robot_state

    try:
        while not rospy.is_shutdown():
            #TODO: 0. Is there obstacle - LiDAR sensor
            # if(lidar_sensor):
            #     turn_to_avoid_obstacle()
            # else:

            #     ROBOT STATES
            #TODO: 1. Search for aruco
            # if(robot_state == 1):
            #     run camera and aruco to see if it finds aruco
            #     if(aruco_found)
            #         robot_state = 2
            #         else:
            #             calculate to turn robot right 
            #             publish the angle of points
            #             run aruco finder

            # elif(robot_state == 2):
            #    go_to_aruco()


            #TODO: 2. go_to_aruco

            #TODO: 3. grab_aruco

            #TODO: 5. get away form aruco's base

            #TODO: 7. move towards goal

            rospy.spin()


            

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node