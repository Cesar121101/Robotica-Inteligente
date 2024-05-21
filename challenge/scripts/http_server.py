#!/usr/bin/env python3.7
from http.server import BaseHTTPRequestHandler, HTTPServer
import grpc
import robot_pb2
import robot_pb2_grpc

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<h1>Bienvenido al servidor gRPC-HTTP</h1>')
        elif self.path == '/llantas':
            llantas_info = self.get_llantas_info()
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(llantas_info.encode())
        elif self.path == '/imagen':
            image_data = self.get_image_data()
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.end_headers()
            self.wfile.write(image_data)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Error 404: Not Found')

    def get_llantas_info(self):
        # Connect to gRPC server and retrieve llantas info
        channel = grpc.insecure_channel('localhost:50051')
        stub = robot_pb2_grpc.RobotServiceStub(channel)
        response = stub.ObtenerLlantas(robot_pb2.Request(query="example"))
        return f'Llanta izquierda: {response.wl}, Llanta derecha: {response.wr}'
    
    def get_image_data(self):
        # Connect to gRPC server and retrieve image data
        channel = grpc.insecure_channel('localhost:50051')
        stub = robot_pb2_grpc.RobotServiceStub(channel)
        response = stub.ObtenerImagen(robot_pb2.Empty())
        return response.data

def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=8080):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Servidor HTTP escuchando en el puerto {port}...')
    httpd.serve_forever()

if __name__ == '__main__':
    run()
