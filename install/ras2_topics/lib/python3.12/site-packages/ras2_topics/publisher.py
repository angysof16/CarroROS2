import rclpy
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def main():
    # Init node
    rclpy.init()

    # Create node
    node = rclpy.create_node('publisher_node')

    # Create publisher
    pub = node.create_publisher(String, "save_topic", 10)

    counter = 0

    message_ros = String() #std_msgs

    while(rclpy.ok()):
        message = str(counter)
        counter += 1
        message_ros.data = message
        pub.publish(message_ros)

    # Destroy node
    node.destroy_node()
    # Shutdown bridge
    rclpy.shutdown()

if __name__ == '__main__':
    main()