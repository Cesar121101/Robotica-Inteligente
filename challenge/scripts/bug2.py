#!/usr/bin/env python
import rospy
import tf_conversions
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16, Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

angle_increment = 0.0
lsr_dists = []
command = Twist()
setpoint = Twist()
poseRobot = Pose()
points = [[0.0, 0.0, "N"], [2.0, 0.0,"N"]]
linear_vel_max = 1.0
angular_vel_max = 3.0
linear_vel_min = 0.05
angular_vel_min = 0.1
points_poses = []
print(points)
superError1 = 0.0
superError2 = 0.0
currentTime = 0.0
prevTime = 0.0
arm_status = 0
prevAngle = 0.0
points_bug2 = []
first_bug2 = True
bug2_num_points = 10

def get_arm_status(msg):
    global arm_status
    arm_status = msg.data

def generate_poses():
    global points
    pointsPoses = []

    for x in points:
        poseA = Pose()
        poseA.position.x = x[0]
        poseA.position.y = x[1]
        if x[2] == "N":
            poseA.orientation.z = "N"
        else:
            poseA.orientation.z = x[2]*(np.pi/180.0)

        print(poseA.orientation.z)
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

    print(robotV)
    print(point)

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

    print(robotV)
    print(point)

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
    
def PID_Position(error):
    global currentTime
    global prevTime
    global superError1

    dt = currentTime-prevTime
    
    # P
    #Kp = rospy.get_param("Kp_Position", "No param found")
    #P = Kp*error
    #P = 0.7*error
    P = 0.5*error

    # I
    superError1 += error * dt
    #Ki = rospy.get_param("Ki_Position", "NO param found")
    #I = superError1*0.0065
    I = 0.0

    # D
    #Kd = rospy.get_param("Kd_Position", "NO param found")
    #D = Kd*((error-prevError)/dt)
    D = 0.0

    #prevError = error

    return (P + I + D)

def PID_Orientation(error):
    global currentTime
    global prevTime
    global superError2

    dt = currentTime-prevTime
    
    # P
    #Kp = rospy.get_param("Kp_Orientation", "No param found")
    #P = Kp*error
    #P = 0.0005*error
    P = 0.8*error

    # I
    superError2 += error * dt
    #Ki = rospy.get_param("Ki_Orientation", "NO param found")
    #I = superError2*0.006
    I = 0.0

    # D
    #Kd = rospy.get_param("Kd_Orientation", "NO param found")
    #D = Kd*((error-prevError)/dt)
    D = 0.0

    #prevError = error

    return (P + I + D) 

def check_around(dists):
    for x in dists:
        if x <= 0.6:
            return False
        
    return True

def check_dir(dists, dist, rangei, rangej):
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
        if dist_point_robot < 0.3:
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
    pub = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist, queue_size=10)

    # For Debug
    position_error_pub = rospy.Publisher("/controller/posError", Float32, queue_size=10)
    position_goal_pub = rospy.Publisher("/controller/posGoal", Float32, queue_size=10)
    position_real_pub = rospy.Publisher("/controller/posReal", Float32, queue_size=10)
    orientation_error_pub = rospy.Publisher("/controller/orientError", Float32, queue_size=10)
    orientation_goal_pub = rospy.Publisher("/controller/orientGoal", Float32, queue_size=10)
    orientation_real_pub = rospy.Publisher("/controller/orientReal", Float32, queue_size=10)
    setpoint_pub = rospy.Publisher("/controller/setpoint", Twist, queue_size=10)

    rospy.Subscriber("/puzzlebot_1/base_controller/odom", Odometry, callback_odom)
    rospy.Subscriber("/puzzlebot_1/scan", LaserScan, callback_lsr)
    rospy.init_node("controller")
    rate = rospy.Rate(100)

    init_command()
    init_setpoint()
    points_poses = generate_poses()
    current_point = 0
    current_state = 0
    new_robot_orientation = 0.0
    prev_position = Pose()
    angular_vel = 0.0
    prevTime = 0.0
    arm_status = 0
    dist_error = 0.0
    dist_goal= 0.0
    angle_goal = 0.0
    angle_error = 0.0
    dist_real = 0.0

    while not rospy.is_shutdown():
        
        robot_position = poseRobot
        (x, y, robot_orientation) = euler_from_quaternion([poseRobot.orientation.x, poseRobot.orientation.y, poseRobot.orientation.z, poseRobot.orientation.w])
        robot_orientation = wrap_to_system(robot_orientation, prevAngle)

        currentTime = rospy.get_time()
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
            print("NEXT POINT")
            print(points)
            print(current_point)
            if current_state >= len(points_poses):
                print("DONE")
            else:
                current_point += 1
                current_state = 1
                superError1 = 0.0
                superError2 = 0.0
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

            print("robot angle: ", robot_orientation)
            print("angle Goal: ", angle_goal)
            print("angle Error: ", angle_error)
            print("angular vel: ", angular_vel)

            rospy.loginfo("robot angle: %f", robot_orientation)
            rospy.loginfo("angle Goal: %f", angle_goal)
            rospy.loginfo("angle Error: %f", angle_error)
            rospy.loginfo("angular vel: %f", angular_vel)

            if np.abs(angle_error) < 0.01:
                print("DONE")
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
            print("LSR: ", lsr_dists[180])
            if check_dir(lsr_dists, 0.3, 150, 211):
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
                    print("DONE")
                    if points_poses[current_point].orientation.z == "N":
                        current_state = 0
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
                print("DONE")
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
                    point_bug2[0] = delta_dist_bug2*x*np.cos(robot_orientation)+robot_position.position.x+0.3*np.cos(robot_orientation)
                    point_bug2[1] = delta_dist_bug2*x*np.sin(robot_orientation)+robot_position.position.y+0.3*np.sin(robot_orientation)
                    points_bug2.append(point_bug2)
                first_bug2 = False
            print(points_bug2)
            angle_indp_bug = get_angle_robot_and_point(robot_orientation, robot_position, points_poses, current_point)
            #print("Angle IND:", angle_indp_bug)
            angle_dir = int(abs(round(180+angle_indp_bug*(180/np.pi))))
            if angle_dir == 360:
                angle_dir = 0
            print("Angle:",angle_dir)
            print("Mes:",lsr_dists[angle_dir])
            
            if bug2_in_track(robot_position,points_bug2) or check_around(lsr_dists):
                # Same code as state 0
                current_state = 1
                superError1 = 0.0
                superError2 = 0.0
                prevAngle = robot_orientation
                new_robot_orientation = robot_orientation
                #first_bug2 = True
            else:
                if check_dir(lsr_dists, 0.3, 145, 226):
                    command.linear.x = 0.0
                    command.angular.z = 0.5
                    print("Left")

                elif not(check_dir(lsr_dists, 0.8, 110, 131)):
                    command.linear.x = 0.075
                    command.angular.z = -0.5
                    print("right")

                elif check_dir(lsr_dists, 0.32, 110, 131):
                    command.linear.x = 1.0
                    command.angular.z = 0.0
                    print("right")
                    

                elif check_dir(lsr_dists, 0.2, 110, 131):
                    command.linear.x = 0.01
                    command.angular.z = 0.3
                    print("left emergency")

                else:
                    command.linear.x = 0.0
                    command.angular.z = -0.5
                    print("forward")
                    

        prevTime = currentTime
        prevAngle = robot_orientation

        pub.publish(command)
        setpoint_pub.publish(setpoint)

        rate.sleep()
