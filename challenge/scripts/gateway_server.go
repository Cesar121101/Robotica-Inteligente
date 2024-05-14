package main

import (
    "fmt"
    "log"
    "net/http"
)

func main() {
    // Manejador para el endpoint "/"
    http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
        fmt.Fprintf(w, "¡Bienvenido al servidor de puerta de enlace!")
    })

    // Manejador para el endpoint "/llantas"
    http.HandleFunc("/llantas", func(w http.ResponseWriter, r *http.Request) {
        // Aquí podrías implementar la lógica para comunicarte con tus servicios y obtener los datos de las llantas
        // Por ahora, solo respondemos con datos de ejemplo
        fmt.Fprintf(w, "Datos de las llantas: wl=0.5, wr=0.7")
    })

    // Iniciamos el servidor en el puerto 8080
    log.Fatal(http.ListenAndServe(":8080", nil))
}
