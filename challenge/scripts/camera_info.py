#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo
import time

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("camera_info")
    print("Camera info is running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    D = [0.17356651064915654, -0.2647240004717277, 0.0026124335567832653, 0.0038337881073672903, 0.0]
    K = [1260.9174439124045, 0.0, 669.9996103566048, 0.0, 1178.106848689801, 407.54013926919983, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P = [1297.4097900390625, 0.0, 673.682553870909, 0.0, 0.0, 1214.8385009765625, 408.92185196993893, 0.0, 0.0, 0.0, 1.0, 0.0]

    #Setup de publishers
    camera_pub = rospy.Publisher("video_source/camera_info", CameraInfo, queue_size=10)

    try:
        while not rospy.is_shutdown():

            camera_info = CameraInfo()
            camera_info.header.frame_id = str(rospy.Time.now())
            camera_info.D = D
            camera_info.K = K
            camera_info.R = R
            camera_info.P = P
            camera_info.height = 1280
            camera_info.width = 720
            camera_info.header.stamp = rospy.Time.now()
            
            camera_pub.publish(camera_info)

    except rospy.ROSInterruptException:
        pass #Initialize and Setup node