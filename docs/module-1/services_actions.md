# ROS 2 Services and Actions: Request-Response and Long-Running Tasks

While ROS 2 Topics provide an asynchronous, publish-subscribe communication mechanism, robotic systems often require synchronous, request-response interactions or a structured way to handle long-running, goal-oriented tasks. This is where ROS 2 Services and Actions come into play.

## Services: Synchronous Request-Response Communication

**Services** in ROS 2 enable a client node to send a request to a service server node and wait for a response. This is a synchronous communication pattern, meaning the client is blocked until it receives a response (or a timeout occurs). Services are ideal for functionalities that require an immediate answer or a single transaction, such as:

*   Querying the state of a sensor (e.g., "What is the current temperature?").
*   Triggering a specific action that completes quickly (e.g., "Open gripper").
*   Requesting a computation (e.g., "Calculate inverse kinematics for this pose").

### Service Definition
A service is defined by a pair of messages: a request message and a response message. These are typically defined in `.srv` files within a ROS 2 package.

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### Example in `rclpy` (Python)
Here's a simplified conceptual example of a service server and client.

#### Service Server Node

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard example service

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Description*: This node acts as a service server, listening for `AddTwoInts` requests. When a request is received, it adds the two integers and sends back their sum.

#### Service Client Node

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    minimal_client.send_request(a, b)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().error(f'Service call failed: {e}')
            else:
                minimal_client.get_logger().info(
                    f'Result of add_two_ints: for {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')
            break
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Description*: This node acts as a service client, sending a request to the `add_two_ints` service and printing the received sum.

## Actions: Long-Running Goal-Oriented Tasks

**Actions** are an extension of services, designed for long-running tasks where feedback on the progress of the goal is required, and the ability to preempt (cancel) the goal is necessary. Actions are built upon ROS 2 Topics and Services. They are crucial for tasks like:

*   Navigating a robot to a distant goal (provides continuous feedback on current position).
*   Executing a complex manipulation sequence (reports progress of each step).
*   Charging a robot (can be cancelled if an emergency occurs).

An action consists of three parts:

1.  **Goal**: The request sent by the client (e.g., "Go to X, Y").
2.  **Feedback**: Intermediate updates on the progress of the goal (e.g., "Robot is at X', Y'").
3.  **Result**: The final outcome of the goal (e.g., "Goal reached" or "Failed to reach goal").

### Action Definition
An action is defined in an `.action` file, specifying the goal, result, and feedback messages.

```
# Goal
int32 order_id
---
# Result
int32[] sequence
---
# Feedback
float32 percentage_complete
```

Actions involve more complex interaction patterns (client, server, and intermediary feedback). Due to this complexity, they are often implemented using dedicated action client and server libraries provided by ROS 2. We will explore a practical example of ROS 2 actions in a later section when building more complex robot behaviors.

In summary, Services are for quick, synchronous request-response needs, while Actions are for complex, long-running tasks that require continuous feedback and potential preemption. These communication primitives provide the flexibility needed for diverse robotic applications.