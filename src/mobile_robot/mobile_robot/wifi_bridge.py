import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import requests
import json

class WifiBridgeNode(Node):
    def __init__(self):
        super().__init__('wifi_bridge')

        # Parámetro para la IP de la ESP32
        self.declare_parameter('esp32_ip', '172.20.10.9')
        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.get_logger().info(f'Conectando a la ESP32 en la IP: {self.esp32_ip}')

        # Suscriptor a /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publicador para los datos del sensor
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_data', 10)

        # Temporizador para solicitar datos del sensor
        timer_period = 0.02  # 10 Hz
        self.timer = self.create_timer(timer_period, self.sensor_request_callback)

        self.esp32_cmd_vel_url = f'http://{self.esp32_ip}:8044/cmd_vel'
        self.esp32_sensor_url = f'http://{self.esp32_ip}:8044/sensor_data'

    def cmd_vel_callback(self, msg):
        """ Envía los comandos de velocidad a la ESP32 por HTTP POST """
        data = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        try:
            requests.post(self.esp32_cmd_vel_url, json=data, timeout=0.05)
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'No se pudo enviar el comando a la ESP32: {e}')

    def sensor_request_callback(self):
        """ Pide datos del sensor y los publica en ROS2 """
        try:
            response = requests.get(self.esp32_sensor_url, timeout=0.5)
            if response.status_code == 200:
                sensor_data = response.json()
                distance_m = sensor_data.get('distance', 0.0) / 100.0 # Convertir cm a metros

                # Publicar como mensaje sensor_msgs/Range
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = 'ultrasonic_sensor_link'
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.26 # Aprox. 15 grados en radianes
                range_msg.min_range = 0.02 # 2 cm
                range_msg.max_range = 4.00 # 4 m
                range_msg.range = float(distance_m)

                self.publisher_.publish(range_msg)
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'No se pudieron obtener datos del sensor: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WifiBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()