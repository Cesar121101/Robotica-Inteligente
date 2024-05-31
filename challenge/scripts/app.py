import grpc
import base64
import threading
import robot_pb2
import robot_pb2_grpc
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import logging

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Configuración del cliente gRPC
channel = grpc.insecure_channel('localhost:50051')
stub = robot_pb2_grpc.RobotServiceStub(channel)

def grpc_client():
    while True:
        # Obtener imagen
        image_response = stub.ObtenerImagen(robot_pb2.Empty())
        image_data = image_response.data
        if image_data:
            # Convierte la imagen a base64 para transmitirla a través de WebSockets
            encoded_image = base64.b64encode(image_data).decode('utf-8')
            socketio.emit('image', {'image': encoded_image}, namespace='/test')
        
        # Obtener datos de las llantas
        llantas_response = stub.ObtenerLlantas(robot_pb2.Empty())
        wl = llantas_response.wl
        wr = llantas_response.wr
        socketio.emit('llantas', {'wl': wl, 'wr': wr}, namespace='/test')

        socketio.sleep(1)  # Espera 1 segundo antes de solicitar la siguiente actualización

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect', namespace='/test')
def test_connect():
    app.logger.info('Cliente conectado')

@socketio.on('disconnect', namespace='/test')
def test_disconnect():
    app.logger.info('Cliente desconectado')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    app.logger.setLevel(logging.INFO)
    # Iniciar la tarea de fondo para el cliente gRPC
    socketio.start_background_task(grpc_client)
    socketio.run(app, host='0.0.0.0', port=5000)
