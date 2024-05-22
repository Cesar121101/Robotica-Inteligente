import grpc
import base64
import threading
import robot_pb2
import robot_pb2_grpc
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)

# Configuración del cliente gRPC
channel = grpc.insecure_channel('localhost:50051')
stub = robot_pb2_grpc.RobotServiceStub(channel)

def grpc_client():
    while True:
        response = stub.ObtenerImagen(robot_pb2.Empty())
        image_data = response.data
        if image_data:
            # Convierte la imagen a base64 para transmitirla a través de WebSockets
            encoded_image = base64.b64encode(image_data).decode('utf-8')
            socketio.emit('image', {'image': encoded_image}, namespace='/test')
        socketio.sleep(1)  # Espera 1 segundo antes de solicitar la siguiente imagen

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect', namespace='/test')
def test_connect():
    print('Cliente conectado')

if __name__ == '__main__':
    # Iniciar la tarea de fondo para el cliente gRPC
    socketio.start_background_task(grpc_client)
    socketio.run(app, host='0.0.0.0', port=5000)
