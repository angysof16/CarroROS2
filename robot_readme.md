# Robot WiFi Controller con ROS2

Este proyecto implementa un robot móvil controlado por WiFi usando ROS2 y un microcontrolador ESP32. El robot puede moverse de forma autónoma y evitar obstáculos usando un sensor ultrasónico.

## Descripción del Proyecto

El sistema está dividido en dos partes principales: el código del ESP32 que controla los motores y sensores del robot, y los nodos de ROS2 que manejan la comunicación y la lógica de evasión de obstáculos.

El robot se conecta a una red WiFi y establece comunicación TCP con el sistema ROS2. Cuando detecta un obstáculo a menos de 5 cm de distancia (configurable), automáticamente se detiene y gira para evitar la colisión.

## Componentes del Hardware

- **2 Motores DC** para el movimiento del robot
- **1 Sensor ultrasónico** para detección de obstáculos  
- **ESP32** como microcontrolador principal
- Chasis del robot con ruedas y sistema de alimentación

## Arquitectura del Software

### ESP32 (main.cpp)
El ESP32 actúa como servidor TCP en el puerto 8080. Simula los datos del sensor ultrasónico enviando mediciones cada 500ms con el formato "U:distancia\n". La distancia simulada varía entre 20 y 100 cm para demostrar la funcionalidad del sistema.

### Nodos ROS2

**WiFi Bridge Node (wifi_bridge.py)**
Este nodo actúa como puente entre ROS2 y el ESP32. Se suscribe al tópico `/cmd_vel` para recibir comandos de velocidad y los envía al robot por HTTP. También solicita datos del sensor cada 50ms y los publica en el tópico `/ultrasonic_data`.

**Obstacle Avoider Node (obstacle_avoider.py)**
Implementa la lógica de evasión de obstáculos. Se suscribe a `/ultrasonic_data` y publica comandos de velocidad en `/cmd_vel`. Cuando detecta un obstáculo, detiene el robot y ejecuta una maniobra de giro de 1 segundo antes de continuar avanzando.

## Configuración e Instalación

### Configuración del ESP32

Antes de cargar el código, modifica las credenciales WiFi en `main.cpp`:

```cpp
const char* ssid = "TU_RED_WIFI";
const char* password = "TU_PASSWORD";
```

Compila y carga el código en tu ESP32. El monitor serial mostrará la dirección IP asignada al dispositivo.

### Configuración ROS2

Navega al directorio del workspace y compila el proyecto:

```bash
cd /ras2_ws
colcon build
source install/setup.bash
```

## Ejecución del Sistema

Para lanzar el sistema completo, usa el archivo launch con la IP de tu ESP32:

```bash
ros2 launch mobile_robot robot_launch.py esp32_ip:='192.168.1.100' obstacle_distance_cm:=7.0
```

Reemplaza `192.168.1.100` con la IP real de tu ESP32. El parámetro `obstacle_distance_cm` permite ajustar la distancia de detección de obstáculos.

## Monitoreo del Sistema

Puedes monitorear los datos del sensor ultrasónico:
```bash
ros2 topic echo /ultrasonic_data
```

Y verificar los comandos de velocidad enviados al robot:
```bash
ros2 topic echo /cmd_vel
```

## Parámetros Configurables

- **esp32_ip**: Dirección IP del ESP32 (default: 192.168.1.100)
- **obstacle_distance_cm**: Distancia en centímetros para activar la evasión (default: 5.0)

## Funcionamiento

El robot avanza continuamente a una velocidad de 0.6 m/s. Cuando el sensor ultrasónico detecta un obstáculo dentro del rango configurado, el sistema ejecuta la siguiente secuencia:

1. Detiene inmediatamente el movimiento hacia adelante
2. Gira hacia la izquierda por 1 segundo
3. Reanuda el movimiento hacia adelante

Este comportamiento permite al robot navegar de forma autónoma evitando colisiones con obstáculos en su camino.

## Desarrollo

Este proyecto fue desarrollado como parte de un curso de robótica usando ROS2. Demuestra conceptos fundamentales como comunicación entre nodos, uso de parámetros, publicación y suscripción a tópicos, y control en tiempo real de robots móviles.