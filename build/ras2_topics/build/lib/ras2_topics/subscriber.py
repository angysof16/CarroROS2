import rclpy
from std_msgs.msg import String

def callback_string(msg):
    print(msg.data)

def main(args=None):
    # Create Node
    rclpy.init(args=args)
    node = rclpy.create_node('subscriptor')

    # Subscriptor
    subscriptor = node.create_subscription(String,'/save_topic', callback_string, 10)
    
    # Keep the node alive
    rclpy.spin(node)

    # Destroy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()