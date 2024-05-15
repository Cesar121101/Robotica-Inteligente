#!/usr/bin/env python
import cv2
import numpy as np

# Definir la matriz de la camara
camera_matrix = np.array([[1260.917444, 0.0, 669.999610], [0.0, 1178.106849, 407.540139], [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.173567, -0.264724, 0.002612, 0.003834, 0.0])

# Cargar el diccionario de marcadores ArUco
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

# Crear parametros del detector
parameters = cv2.aruco.DetectorParameters_create()

# Leer la imagen con el marcador ArUco
image_path = 'aruco.jpg'
image = cv2.imread(image_path)

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
        print("ID:", ids[i][i])
        print("Position:", tvecs[i][i])

# Mostrar la imagen con los marcadores detectados y los ejes de coordenadas
cv2.imshow('ArUco Markers', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
