# Example: Deploying a Simple ROS 2 Node to Jetson Orin

This example demonstrates a basic ROS 2 Python node designed to run on an NVIDIA Jetson Orin device. The primary goal is to illustrate the simplicity of running `rclpy` applications on the Jetson after the development environment has been correctly set up and the workspace built.

## Goal

To deploy and execute a minimal ROS 2 publisher node on an NVIDIA Jetson Orin, confirming basic ROS 2 functionality on the edge device.

## Prerequisites

*   **NVIDIA Jetson Orin Nano/NX**: With JetPack 6.1 and ROS 2 Humble installed (as per [Installation Guides](../../docs/appendices/install-guides.md)).
*   **Development PC**: Where this node was developed (optional, but typical workflow).
*   **ROS 2 Workspace**: A ROS 2 workspace created on the Jetson, with necessary packages built.

## Example File

### `jetson_publisher.py` (Python ROS 2 Node)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class JetsonPublisher(Node):
    def __init__(self):
        super().__init__('jetson_publisher')
        self.publisher_ = self.create_publisher(String, 'jetson_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Jetson Publisher Node has started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Jetson! Message count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    jetson_publisher = JetsonPublisher()
    try:
        rclpy.spin(jetson_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        jetson_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Deployment and Execution Steps (on Jetson Orin)

1.  **Transfer the Package**:
    *   On your development PC, copy the entire ROS 2 package containing `jetson_publisher.py` (e.g., `my_jetson_pkg`) to your Jetson Orin. You can use `scp`:
        ```bash
        # From your development PC's terminal
        scp -r /path/to/my_jetson_pkg/ username@jetson_ip_address:~/ros2_ws/src/
        ```
2.  **Access Jetson Terminal**: Open a terminal on your Jetson Orin (e.g., via SSH or directly).
3.  **Navigate to Workspace**:
    ```bash
    cd ~/ros2_ws
    ```
4.  **Install Dependencies (if any)**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
5.  **Build Workspace**:
    ```bash
    colcon build --packages-select my_jetson_pkg # Replace my_jetson_pkg with your package name
    ```
6.  **Source Workspace**:
    ```bash
    source install/setup.bash
    ```
7.  **Run the Node**:
    ```bash
    ros2 run my_jetson_pkg jetson_publisher # Replace my_jetson_pkg
    ```
8.  **Verify (on Jetson or Development PC)**:
    *   In a new terminal (on Jetson or a connected development PC, if `ROS_DOMAIN_ID` is set correctly for network communication):
        ```bash
        ros2 topic list
        ros2 topic echo /jetson_topic
        ```
        You should see "Hello from Jetson!" messages being published.

This example confirms that your Jetson Orin is correctly configured for ROS 2 development and can execute Python-based robotics applications at the edge.