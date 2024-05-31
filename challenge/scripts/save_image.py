#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Int16

image = Image()

def camera_callback(msg): 
    global image
    bridge = CvBridge()

    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Mostrar la imagen con los marcadores detectados y los ejes de coordenadas
    image = cv2.resize(image, (640, 480))
    cv2.imwrite("src/challenge/image.jpg", image)
    # cv2.imshow('ArUco Markers', image)

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("save_image")
    print("Save image is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup suscribers
    rospy.Subscriber("video_source/raw", Image, camera_callback)

    while not rospy.is_shutdown():

        # cv2.destroyAllWindows()
        rospy.spin()
