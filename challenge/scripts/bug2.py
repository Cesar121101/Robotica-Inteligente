#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16, Float32, Float64MultiArray
from tf.transformations import euler_from_quaternion
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

angle_increment = 0.0
lsr_dists = []
command = Twist()
setpoint = Twist()
poseRobot = Pose()
points = []
points_msg = Float64MultiArray()
robot_state_msg = 0
generated_poses = 0
linear_vel_max = 0.15
angular_vel_max = 2.0
linear_vel_min = 0.05
angular_vel_min = 0.1
points_poses = []
arm_status = 0
prevAngle = 0.0
points_bug2 = []
first_bug2 = True
bug2_num_points = 10
robot_state_flag = 0
state5_flag = 0

def get_arm_status(msg):
    global arm_status
    arm_status = msg.data

def generate_poses():
    global points
    pointsPoses = []
    poseA = Pose()
    for x in points:
        poseA.position.x = x[0]
        poseA.position.y = x[1]
        if x[2] == "N":
            poseA.orientation.z = "N"
        else:
            poseA.orientation.z = x[2]*(np.pi/180.0)

        pointsPoses.append(poseA)

    return pointsPoses

def get_angle_robot_and_point(robot_orientation,robot_position, points_poses, current_point):
    vectorL = 1
    robotV = [0.0, 0.0]
    point = [0.0,0.0]
    working_point = points_poses[current_point]

    robotV[0] = vectorL*np.cos(robot_orientation)
    robotV[1] = vectorL*np.sin(robot_orientation)

    point[0] = working_point.position.x - robot_position.position.x
    point[1] = working_point.position.y - robot_position.position.y

    angle = np.arccos(np.dot(robotV, point)/(np.sqrt((point[0]*point[0]) + (point[1]*point[1]))))
    cross = np.cross(robotV, point)

    if cross < 0:
        angle = angle * -1.0
    elif cross > 0:
        angle = angle
    else:
        angle = 0.0
    
    return angle

def wrap_to_system(angle, prevAngle):
    if angle > np.pi/2 and prevAngle < -np.pi/2:
        return angle-2*np.pi
    elif angle < -np.pi/2 and prevAngle > np.pi/2:
        return angle + 2*np.pi
    else:
        return angle

def get_angle_robot_and_orientation(robot_orientation, points_poses, current_point):
    global setpoint
    vectorL = 1
    robotV = [0.0, 0.0]
    point = [0.0,0.0]
    working_point = points_poses[current_point]

    robotV[0] = vectorL*np.cos(robot_orientation)
    robotV[1] = vectorL*np.sin(robot_orientation)

    point[0] = vectorL*np.cos(working_point.orientation.z)
    point[1] = vectorL*np.sin(working_point.orientation.z)

    angle = np.arccos(np.dot(robotV, point)/(np.sqrt((point[0]*point[0]) + (point[1]*point[1]))))
    cross = np.cross(robotV, point)

    if cross < 0:
        angle = angle * -1.0
    elif cross > 0:
        angle = angle
    else:
        angle = 0.0
    
    return angle

def dist_poses(pose1, pose2):
    dist = np.sqrt((np.power((pose2.position.x - pose1.position.x), 2) + np.power((pose2.position.y - pose1.position.y), 2)))
    
    return dist

def callback_lsr(msg):
    global lsr_dists, angle_increment
    angle_increment = msg.angle_increment
    lsr_dists = msg.ranges

def callback_odom(msg):
    global poseRobot
    poseRobot = msg.pose.pose

def callback_points(msg):
    global points_msg, points, poseRobot
    points_msg = msg.data
    points = [[poseRobot.position.x, poseRobot.position.y, "N"], [points_msg[0], points_msg[1], "N"]]
    

def callback_robot_state(msg):
    global robot_state_msg
    robot_state_msg = msg.data
    
def PID_Position(error):
    P = 0.5*error
    I = 0.0
    D = 0.0

    return (P + I + D)

def PID_Orientation(error):
    P = 0.8*error
    I = 0.0
    D = 0.0

    return (P + I + D) 

def check_around(dists):
    for x in dists:
        if x <= 0.6:
            return False
        
    return True

def delete_path(points, robot_position):
    new_list = []
    for x in points:
        # Dist to point
        robot_x = robot_position.position.x
        robot_y = robot_position.position.y

        point_x = x[0] #X
        point_y = x[1] #Y

        dist_point_robot = np.sqrt((np.power((point_x - robot_x), 2) + np.power((point_y - robot_y), 2)))
        if dist_point_robot > 0.15:
            new_list.append(x)
    return new_list

def check_dir(dists, dist, rangei, rangej):
    rangei = int(round(rangei/0.314))
    rangej = int(round(rangej/0.314))
    for x in range(rangei,rangej):
        if dists[x] <= dist:
            return True
        
    return False

def bug2_in_track(robot_position, points):
    for x in points:
        # Dist to point
        robot_x = robot_position.position.x
        robot_y = robot_position.position.y

        point_x = x[0] #X
        point_y = x[1] #Y

        dist_point_robot = np.sqrt((np.power((point_x - robot_x), 2) + np.power((point_y - robot_y), 2)))
        if dist_point_robot < 0.15:
            return True
        
    return False

def init_command():
    global command
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

def init_setpoint():
    global setpoint
    setpoint.linear.x = 0.0
    setpoint.linear.y = 0.0
    setpoint.linear.z = 0.0
    setpoint.angular.x = 0.0
    setpoint.angular.y = 0.0
    setpoint.angular.z = 0.0

if __name__ == '__main__':
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # For Debug
    position_error_pub = rospy.Publisher("/controller/posError", Float32, queue_size=10)
    position_goal_pub = rospy.Publisher("/controller/posGoal", Float32, queue_size=10)
    position_real_pub = rospy.Publisher("/controller/posReal", Float32, queue_size=10)
    orientation_error_pub = rospy.Publisher("/controller/orientError", Float32, queue_size=10)
    orientation_goal_pub = rospy.Publisher("/controller/orientGoal", Float32, queue_size=10)
    orientation_real_pub = rospy.Publisher("/controller/orientReal", Float32, queue_size=10)
    setpoint_pub = rospy.Publisher("/controller/setpoint", Twist, queue_size=10)
    robot_state_flag_pub = rospy.Publisher("state_flag", Int16, queue_size=10)
    state5_flag_pub = rospy.Publisher("state5_flag", Int16, queue_size=10)
    rospy.init_node("controller")

    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/scan", LaserScan, callback_lsr)
    rospy.Subscriber("/points", Float64MultiArray, callback_points)
    rospy.Subscriber("/state", Int16, callback_robot_state)
    rate = rospy.Rate(100)

    init_command()
    init_setpoint()
    points_poses = generate_poses()
    current_point = 0
    current_state = 0
    new_robot_orientation = 0.0
    prev_position = Pose()
    angular_vel = 0.0
    arm_status = 0
    dist_error = 0.0
    dist_goal= 0.0
    angle_goal = 0.0
    angle_error = 0.0
    dist_real = 0.0
    back_counter = 0

    while not rospy.is_shutdown():
        # points = points_msg
        print("Puntos: ", points)
        print("Estado del robot", robot_state_msg)

        #? Checking it point to reach change
        if(generated_poses == 0 and (robot_state_msg == 2 or robot_state_msg == 5 or robot_state_msg == 7) and len(points) > 0):
            generated_poses = 1
            points_poses = generate_poses()
            print("generar puntos")

        # TODO: 1. Search for aruco
        if(robot_state_msg == 1 or robot_state_msg == 6):
            if(robot_state_flag == 0): # aruco.py changes state flag
                command.linear.x = 0.0
                command.angular.z = 0.3
            else:
                command.linear.x = 0.0
                command.angular.z = 0.0
            current_state = 0
            
        # TODO: 2. go to aruco or to finishing point
        elif(points > 0 and (robot_state_msg == 2 or robot_state_msg == 5 or robot_state_msg == 7)):

            robot_state_flag = 0
            state5_flag = 0
            
            robot_position = poseRobot
            (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])
            robot_orientation = wrap_to_system(robot_orientation, prevAngle)

            print("Current Point: ", current_point)

            # publish errors and goals to see graphs
            position_error_pub.publish(dist_error)
            position_goal_pub.publish(dist_goal)
            position_real_pub.publish(dist_real)
            orientation_error_pub.publish(angle_error)
            orientation_goal_pub.publish(angle_goal)
            orientation_real_pub.publish(robot_orientation)
            #setpoint_pub.publish(setpoint)
            setpoint.angular.z = angle_goal*180/np.pi  
            
            if current_state == 0:
                if current_state >= len(points_poses):
                    print("DONE-1")
                else:
                    current_point += 1
                    current_state = 1
                    setpoint.linear.x = points_poses[current_point].position.x
                    setpoint.linear.y = points_poses[current_point].position.y
                    setpoint.linear.z = 0.0
                    setpoint.angular.x = 0.0
                    setpoint.angular.y = 0.0
                    #setpoint.angular.z = angle_goal*180/np.pi
                prevAngle = robot_orientation
                new_robot_orientation = robot_orientation
            
            elif current_state == 1:
                print("GETTING ANGLE")

                angle_indp = get_angle_robot_and_point(robot_orientation, robot_position, points_poses, current_point)
                angle_goal = new_robot_orientation + angle_indp  

                current_state = 2

            elif current_state == 2:
                print("SETTING ANGLE")
                angle_error = angle_goal - robot_orientation
                angular_vel = PID_Orientation(angle_error)

                if np.abs(angular_vel) > angular_vel_max:
                    if angular_vel > 0:
                        angular_vel = angular_vel_max
                    else:
                        angular_vel = -1.0*angular_vel_max

                if np.abs(angular_vel) < angular_vel_min:
                    if angular_vel > 0:
                        angular_vel = angular_vel_min
                    else:
                        angular_vel = -1.0*angular_vel_min

                if np.abs(angle_error) < 0.05:
                    print("DONE-2")
                    current_state = 3
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                    prev_position = robot_position
                else:

                    command.linear.x = 0.0
                    command.angular.z = angular_vel

            elif current_state == 3:
                print("GETTING DISTANCE")
                dist_goal = dist_poses(robot_position, points_poses[current_point])
                current_state = 4

            elif current_state == 4:
                print("MOVING")
                points_bug2 = delete_path(points_bug2,robot_position)

                if check_dir(lsr_dists, 0.4, 0, 30) or check_dir(lsr_dists, 0.4, 330, 360) and robot_state_msg != 7:
                    current_state = 7
                else:

                    dist_real = dist_poses(robot_position, prev_position)

                    dist_error = dist_goal - dist_real
                    linear_vel = PID_Position(dist_error)*0.5

                    if np.abs(linear_vel) > linear_vel_max:
                        if linear_vel > 0:
                            linear_vel = linear_vel_max
                        else:
                            linear_vel = -1.0* linear_vel_max

                    if np.abs(linear_vel) < linear_vel_min:
                        if linear_vel > 0:
                            linear_vel = linear_vel_min
                        else:
                            linear_vel = -1.0* linear_vel_min

                    print("robot dist: ", dist_real)
                    print("dist Goal: ", dist_goal)
                    print("dist Error: ", dist_error)
                    print("linear vel: ", linear_vel)

                    if np.abs(dist_error) < 0.002:
                        print("DONE-3")
                        if points_poses[current_point].orientation.z == "N":
                            current_state = 8
                        else:
                            current_state = 5
                            superError2 = 0.0
                            new_robot_orientation = robot_orientation

                        command.linear.x = 0.0
                        command.angular.z = 0.0
                    else:
                        command.linear.x = linear_vel
                        command.angular.z = 0.0

            elif current_state == 5:
                print("ADJUSTING GETTING ANGLE")

                angle_indp_adj = get_angle_robot_and_orientation(robot_orientation, points_poses, current_point)
                angle_goal_adj = new_robot_orientation + angle_indp_adj

                current_state = 6

            elif current_state == 6:
                print("ADJUSTING SETTING ANGLE")
                angle_error_adj = angle_goal_adj - robot_orientation
                angular_vel_adj = PID_Orientation(angle_error_adj)

                if np.abs(angular_vel_adj) > angular_vel_max:
                    if angular_vel_adj > 0:
                        angular_vel_adj = angular_vel_max
                    else:
                        angular_vel_adj = -1.0*angular_vel_max

                if np.abs(angular_vel_adj) < angular_vel_min:
                    if angular_vel_adj > 0:
                        angular_vel_adj = angular_vel_min
                    else:
                        angular_vel_adj = -1.0*angular_vel_min

                print("robot angle: ", robot_orientation)
                print("angle Goal: ", angle_goal_adj)
                print("angle Error: ", angle_error_adj)
                print("angular vel: ", angular_vel_adj)

                if np.abs(angle_error_adj) < 0.0018:
                    print("DONE-4")
                    current_state = 0
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                    prev_position = robot_position
                else:

                    command.linear.x = 0.0
                    command.angular.z = angular_vel_adj

            elif current_state == 7:
                print("BUG2")
                if first_bug2:
                    delta_dist_bug2 = dist_error/bug2_num_points
                    points_bug2 = []
                    # Populate points
                    for x in range (1, bug2_num_points+1):
                        point_bug2 = [0.0, 0.0]
                        point_bug2[0] = delta_dist_bug2*x*np.cos(robot_orientation)+robot_position.position.x+0.4*np.cos(robot_orientation)
                        point_bug2[1] = delta_dist_bug2*x*np.sin(robot_orientation)+robot_position.position.y+0.4*np.sin(robot_orientation)
                        points_bug2.append(point_bug2)
                    first_bug2 = False
                print(points_bug2)
                angle_indp_bug = get_angle_robot_and_point(robot_orientation, robot_position, points_poses, current_point)
                angle_dir = int(abs(round((angle_indp_bug*(180/np.pi))/0.314)))
                if angle_dir == 360:
                    angle_dir = 0
                print("Angle:",angle_dir)
                print("Mes:",lsr_dists[angle_dir])
                
                if bug2_in_track(robot_position,points_bug2):
                    # Same code as state 0
                    current_state = 1
                    prevAngle = robot_orientation
                    new_robot_orientation = robot_orientation
                else:
                    # Left
                    if check_dir(lsr_dists, 0.4, 0, 36) or check_dir(lsr_dists, 0.4, 325, 360): 
                        command.linear.x = 0.0
                        command.angular.z = 0.75

                    elif check_dir(lsr_dists, 0.5, 36, 66):
                        command.linear.x = 0.3
                        command.angular.z = -0.3

                    # Right
                    elif not(check_dir(lsr_dists, 0.9, 65, 76)):
                        command.linear.x = 0.3
                        command.angular.z = -0.35

                    #Forward
                    elif check_dir(lsr_dists, 0.35, 65, 76):
                        command.linear.x = 0.4
                        command.angular.z = 0.0
                        
                    # Emergency
                    elif check_dir(lsr_dists, 0.3, 65, 76):
                        command.linear.x = 0.2
                        command.angular.z = 0.5

                    # Final right
                    else:
                        command.linear.x = 0.18
                        command.angular.z = -0.75
            # Route finalized   
            elif current_state == 8:
                print("DONE-5")
                command.linear.x = 0.0
                command.angular.z = 0.0

                if robot_state_msg == 2:
                    robot_state_flag = 1
                    current_point = 0
                    current_state = 0
                    points = []
                    generated_poses = 0
                elif robot_state_msg == 5:
                    state5_flag = 1
                    current_point = 0
                    points = []
                    generated_poses = 0
                elif robot_state_msg == 7:
                    robot_state_flag = 1
                else:
                    robot_state_flag = 0
                    state5_flag = 0

            prevAngle = robot_orientation

            setpoint_pub.publish(setpoint)
            robot_state_flag_pub.publish(robot_state_flag)
            state5_flag_pub.publish(state5_flag)

        # TODO: Make Puzzlebot move backwards
        elif(robot_state_msg == 4):
            robot_state_flag = 0
            if (back_counter <= 70):
                print("MOVE AWAY ARUCO")
                command.linear.x = -0.2
                command.linear.y = 0.0
                command.linear.z = 0.0
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = 0.0
                back_counter += 1
            elif (back_counter > 70):
                command.linear.x = 0.0
                robot_state_flag = 1
            robot_state_flag_pub.publish(robot_state_flag)
        
        #TODO: 8. leave crate
        elif(robot_state_msg == 8):
            init_command()
        
        else:
            print("No points were found... perhaps it finished.")
            robot_state_flag = 0
            robot_state_flag_pub.publish(robot_state_flag)
            state5_flag_pub.publish(state5_flag)
            
        pub.publish(command)
        print(command)
        
        rate.sleep()