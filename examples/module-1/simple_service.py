import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # ROS 2 standard example service

# --- Service Server Node ---
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # Create a service named 'add_two_ints' of type AddTwoInts
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('ROS 2 Service Server "add_two_ints" started.')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the service.
        It receives a request, performs an operation (addition), and fills the response.
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

# --- Service Client Node ---
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available before trying to call it
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "add_two_ints" not available, waiting...')
        self.req = AddTwoInts.Request() # Create an empty request object

    def send_request(self, a, b):
        """
        Sends a request to the service and stores the future object.
        """
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Sending request: a={a}, b={b}')


def main_service_server(args=None):
    """Entry point for the service server."""
    rclpy.init(args=args)
    minimal_service = MinimalService()
    try:
        rclpy.spin(minimal_service) # Keep the service server alive
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

def main_service_client(args=None):
    """Entry point for the service client."""
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    # Get values for a and b from command line arguments or use defaults
    import sys
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    minimal_client.send_request(a, b)

    # Spin until the future is complete
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().error(f'Service call failed: {e}')
            else:
                minimal_client.get_logger().info(
                    f'Result of add_two_ints: {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')
            break # Exit loop after receiving response or error

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # This example demonstrates both server and client in a single file for simplicity.
    # In a real ROS 2 setup, these would typically be run as separate processes.
    # To run the server: python simple_service.py server
    # To run the client: python simple_service.py client 5 3

    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'server':
        main_service_server()
    elif len(sys.argv) > 1 and sys.argv[1] == 'client':
        # Remove 'client' argument before passing to main_service_client
        sys.argv.pop(1)
        main_service_client(args=sys.argv)
    else:
        print("Usage: python simple_service.py [server|client [a b]]")
        print("Example: python simple_service.py server")
        print("Example: python simple_service.py client 5 3")
        rclpy.logging.get_logger("simple_service").info("Please specify 'server' or 'client' argument.")

