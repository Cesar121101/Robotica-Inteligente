#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16

# Definir la matriz de la camara
camera_matrix = np.array([[1260.917444, 0.0, 669.999610], [0.0, 1178.106849, 407.540139], [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.173567, -0.264724, 0.002612, 0.003834, 0.0])

# Cargar el diccionario de marcadores ArUco
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

# Crear parametros del detector
parameters = cv2.aruco.DetectorParameters_create()

image = Image()

pose = Pose()
id = -1
state = -1
state_flag = 0

def camera_callback(msg): 
    global image, id
    global state_flag
    bridge = CvBridge()

    print("antes del if")
    print("State: ", state)
    if state == 1:
        print("  ----RESETING STATE FLAG TO FALSE----  ")
        state_flag = 0 #reseting state_flag

        print("despues del if")
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Obtener las dimensiones de la imagen
        height, width = image.shape[:2]

        # Calcular las coordenadas para el recorte central
        crop_x_start = int(width * 0.25)
        crop_x_end = int(width * 0.75)
        crop_y_start = 0
        crop_y_end = int(height)

        # Recortar la imagen al centro
        image = image[crop_y_start:crop_y_end, crop_x_start:crop_x_end]

        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores ArUco en la imagen
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, arucoDict, parameters=parameters)

        # Dibujar los marcadores detectados en la imagen y obtener la posicion
        if ids is not None and len(ids) > 0:
            # Dibujar los marcadores detectados
            cv2.aruco.drawDetectedMarkers(image, corners)

            # Obtener la posicion y orientacion de los marcadores
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

            # Dibujar ejes de coordenadas 3D para cada marcador
            for i in range(len(ids)):
                cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

            # Mostrar la posicion de cada marcador ArUco
            for i in range(len(ids)):
                id = ids[i][i]
                pose.position.x = tvecs[i][i][0]
                pose.position.y = tvecs[i][i][1]
                pose.position.z = tvecs[i][i][2]
                print("ID:", ids[i][i])
                print("Position:", tvecs[i][i])
                state_flag = 1

                # id_pub.publish(id)
                # pose_pub.publish(pose)
                # state_flag_pub.publish(True)
            id_pub.publish(id)
            pose_pub.publish(pose)
            robot_state_flag_pub.publish(state_flag)    # ARUCO was found
            print("--------POSITION OF THE ARUCO----------")
            print(pose)


        # Mostrar la imagen con los marcadores detectados y los ejes de coordenadas
        # cv2.imwrite("/home/laptop/8_Semestre/catkin_ws_equipo/src/challenge/image.jpg", image)
        # cv2.imshow('ArUco Markers', image)

def callback_state(msg):
    global state
    state = msg.data

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("arucos")
    print("Detect arucos is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup suscribers
    rospy.Subscriber("video_source/raw", Image, camera_callback)
    rospy.Subscriber("/state", Int16, callback_state)

    #Setup publishers
    id_pub = rospy.Publisher("aruco_id", Int16, queue_size=10)
    pose_pub = rospy.Publisher("aruco_pose", Pose, queue_size=10)
    robot_state_flag_pub = rospy.Publisher("state_flag", Int16, queue_size=10)

    while not rospy.is_shutdown():

        # cv2.destroyAllWindows()


        rospy.spin()
