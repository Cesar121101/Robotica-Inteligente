package main

import (
    "context"
    "fmt"
    "log"

    "google.golang.org/grpc"
    pb "protos" // Importa el paquete generado por el compilador protobuf
)

func main() {
    // Establece la dirección del servidor gRPC
    serverAddress := "localhost:50051"

    // Crea una conexión gRPC con el servidor
    conn, err := grpc.Dial(serverAddress, grpc.WithInsecure())
    if err != nil {
        log.Fatalf("No se pudo conectar al servidor: %v", err)
    }
    defer conn.Close()

    // Crea un cliente para el servicio gRPC
    client := pb.NewRobotServiceClient(conn)

    // Llama al método ObtenerLlantas del servicio gRPC
    response, err := client.ObtenerLlantas(context.Background(), &pb.Empty{})
    if err != nil {
        log.Fatalf("Error al llamar al método ObtenerLlantas: %v", err)
    }

    // Imprime los datos recibidos del servicio
    fmt.Printf("Datos de las llantas recibidos: wl=%.2f, wr=%.2f\n", response.Wl, response.Wr)
}
