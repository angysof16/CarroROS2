import rclpy
import sys

from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('node_client')
    #Create Client
    client = node.create_client(AddTwoInts, 'add_two_ints')
    #Wait for service
    while not client.wait_for_service(timeout_sec= 1.0):
        print("Waiting for server")
    #Call Reponse
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    request = AddTwoInts.Request()
    request.a = a
    request.b = b
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    print("sum is: " + str(response.sum))

    rclpy.shutdown()

if __name__ == '__main__':
    main()
