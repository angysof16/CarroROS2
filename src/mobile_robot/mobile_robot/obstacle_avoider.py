import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import time

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Parámetro para la distancia de detección
        self.declare_parameter('obstacle_distance_cm', 5.0)
        self.obstacle_distance_m = self.get_parameter('obstacle_distance_cm').get_parameter_value().double_value / 100.0

        # Suscriptor al sensor
        self.subscription = self.create_subscription(
            Range,
            'ultrasonic_data',
            self.sensor_callback,
            10)

        # Publicador de comandos de velocidad
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estado del robot
        self.is_stopping = False
        self.turn_start_time = 0
        self.turn_duration = 1 # Segundos para girar

        self.get_logger().info(f'Nodo de evasión iniciado. Distancia de parada: {self.obstacle_distance_m*100:.1f} cm')

    def sensor_callback(self, msg):
        current_distance = msg.range

        cmd = Twist()

        if self.is_stopping:
            # Estamos en medio de una maniobra de giro
            if time.time() - self.turn_start_time < self.turn_duration:
                # Sigue girando
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6 # Velocidad de giro positiva (izquierda)
            else:
                # Termina de girar y vuelve al modo normal
                self.is_stopping = False
                cmd.linear.x = 0.6 # Avanza de nuevo
                cmd.angular.z = 0.0
        elif current_distance <= self.obstacle_distance_m and current_distance > 0.0:
            # Obstáculo detectado! Inicia la maniobra
            self.get_logger().info(f'¡Obstáculo detectado a {current_distance:.2f}m! Deteniendo y girando.')
            self.is_stopping = True
            self.turn_start_time = time.time()

            # Detente inmediatamente
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # No hay obstáculos, avanza
            cmd.linear.x = 0.6 # Velocidad de avance constante
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()