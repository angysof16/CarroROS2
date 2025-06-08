from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Argumento para la IP de la ESP32
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip',
        default_value='192.168.1.100', # Valor por defecto
        description='Dirección IP del robot ESP32'
    )

    # Argumento para la distancia de detección
    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance_cm',
        default_value='5.0',
        description='Distancia en cm para detenerse ante un obstáculo'
    )

    # Nodo Puente WiFi
    wifi_bridge_node = Node(
        package='mobile_robot',
        executable='wifi_bridge',
        name='wifi_bridge',
        parameters=[{'esp32_ip': LaunchConfiguration('esp32_ip')}]
    )

    # Nodo de Evasión de Obstáculos
    obstacle_avoider_node = Node(
        package='mobile_robot',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        parameters=[{'obstacle_distance_cm': LaunchConfiguration('obstacle_distance_cm')}]
    )

    return LaunchDescription([
        esp32_ip_arg,
        obstacle_distance_arg,
        wifi_bridge_node,
        obstacle_avoider_node
    ])