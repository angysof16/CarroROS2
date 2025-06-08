import rclpy

from example_interfaces.srv import AddTwoInts

def callback_for_service(request, response):
    print(str(request.a) + " + " + str(request.b) + " = " + str(request.a + request.b))
    response.sum = request.a + request.b
    return response

def main(args=None):
    #Create rlcpy
    rclpy.init(args=None)
    #Create Node
    node = rclpy.create_node('server_node')
    #......SERVICE
    srv = node.create_service(AddTwoInts, 'add_two_ints', callback_for_service)
    #Block Node
    rclpy.spin(node)
    #Destroy    
    rclpy.shutdown()

if __name__ == '__main__':
    main()