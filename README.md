# Embebidos
# Robot10 — Sistema robótico autónomo con brazo MeArm y visión artificial

Este repositorio contiene el código fuente del proyecto final de **Sistemas Embebidos**, correspondiente a un robot móvil diferencial equipado con un brazo robótico tipo **MeArm**, una cámara **ESP32-CAM**, sensores de distancia y batería, comunicación distribuida mediante **PubSub + Broker**, y una interfaz web con visualización, control manual, telemetría y gemelos digitales.

El objetivo principal del sistema es realizar el reto de manipulación autónoma de una bandeja de cartón, donde el robot debe posicionarse frente al objeto, introducir una horquilla debajo de la bandeja, levantarla y desplazarla hacia una zona ubicada aproximadamente 10 cm a la derecha de su posición inicial.

## Descripción general

El sistema está compuesto por tres bloques principales:

1. **Robot móvil con Raspberry Pi Pico W**
   - Controla el carro diferencial.
   - Controla el brazo MeArm.
   - Lee sensores de distancia y batería.
   - Ejecuta comandos recibidos desde el broker.
   - Publica estados y telemetría.

2. **Broker en PC**
   - Centraliza la comunicación PubSub.
   - Recibe mensajes desde la Pico W.
   - Envía comandos desde el dashboard al robot.
   - Integra la tarea de visión artificial.
   - Coordina la misión autónoma.

3. **Dashboard web**
   - Permite control manual del carro y del brazo.
   - Muestra sensores, estados y logs.
   - Visualiza cámara y detección del Tag.
   - Incluye gemelo digital del carro y del brazo.
   - Permite iniciar, pausar, reanudar o abortar la misión.

## Arquitectura del sistema

```text
ESP32-CAM
   │
   │ Video /stream
   ▼
PC Broker + VisionTask
   │
   ├── Procesamiento ArUco
   ├── Publicación vision/estado
   ├── Comunicación TCP con Pico W
   └── Comunicación WebSocket con Dashboard
          ▲
          │
Dashboard Web
          │
          ▼
Raspberry Pi Pico W
   ├── Carro diferencial
   ├── Brazo MeArm
   ├── Sensor VL53L0X
   ├── Monitor de batería
   └── Pantalla OLED
