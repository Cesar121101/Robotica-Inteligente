#!/usr/bin/env python3.7
import cv2
import signal
import threading

from concurrent import futures
import grpc
import robot_pb2
import robot_pb2_grpc

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

wl_global = 0.0
wr_global = 0.0
image = Image()

class RobotService(robot_pb2_grpc.RobotServiceServicer):
    def ObtenerLlantas(self, request, context):
        global wl_global, wr_global
        # Aqui simulamos obtener los datos de las llantas del robot
        # Puedes sustituir esta logica con la real
        return robot_pb2.Llanta(wl=wl_global, wr=wr_global)
    
    def ObtenerImagen(self, request, context):
        # global image_data
        # Simulacion de captura de imagen
        # image = np.zeros((480, 640, 3), dtype=np.uint8)  # Imagen negra de ejemplo
        with open('/home/laptop/8_Semestre/catkin_ws_equipo/src/challenge/image.jpg', 'rb') as f:
            image_data = f.read()
        return robot_pb2.Imagen(data=image_data)
    
# Getting info
def callback_wl(msg):
    global wl_global
    wl_global = msg.data

# Getting info
def callback_wr(msg):
    global wr_global
    wr_global = msg.data

terminate = threading.Event()
def callback_terminate(signum, frame):
    rospy.signal_shutdown("Ending ROS Node")
    terminate.set()

if __name__ == '__main__':

    # Terminate signal
    signal.signal(signal.SIGINT, callback_terminate)

    # Node initialization
    rospy.init_node('rosServer', anonymous=True)
    rate = rospy.Rate(100)

    # Subscribers
    wl_sub = rospy.Subscriber("/wl", Float32, callback_wl)
    wr_sub = rospy.Subscriber("/wr", Float32, callback_wr)
    
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_pb2_grpc.add_RobotServiceServicer_to_server(RobotService(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    print("Servidor iniciado en el puerto 50051...")

    #Wait and repeat
    rospy.spin()
    terminate.wait()
    server.stop(1).wait()
