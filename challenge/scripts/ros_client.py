import grpc
import robot_pb2
import robot_pb2_grpc

def obtener_datos_llantas():
    channel = grpc.insecure_channel('localhost:50051')
    stub = robot_pb2_grpc.RobotServiceStub(channel)
    response = stub.ObtenerLlantas(robot_pb2.Request(query="example"))
    return response

if __name__ == '__main__':
    datos_llantas = obtener_datos_llantas()
    print("Datos de las llantas recibidos:")
    print("WL:", datos_llantas.wl)
    print("WR:", datos_llantas.wr)
