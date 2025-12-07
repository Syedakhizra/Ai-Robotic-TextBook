# Mini-Lab 1: Building a Multi-Node ROS 2 System to Control a Simulated Light

This mini-lab challenges you to integrate the ROS 2 concepts of nodes, topics, publishers, and subscribers to create a multi-node system. Your goal is to simulate a simple "light control" system where one node publishes commands, and another node subscribes to those commands to change the state of a simulated light.

## Learning Objectives

*   Reinforce understanding of ROS 2 topics and message passing.
*   Practice creating multiple ROS 2 nodes that communicate with each other.
*   Develop a basic understanding of system integration in ROS 2.

## Scenario

Imagine a simple home automation system where you want to control a smart light using voice commands. For this mini-lab, we'll simplify it:

*   **Command Node**: This node will simulate a voice command system by publishing `String` messages (e.g., "ON", "OFF") to a `light_command` topic.
*   **Light Node**: This node will subscribe to the `light_command` topic and print the current state of the simulated light (e.g., "Light is ON", "Light is OFF") to the console.

## Task Breakdown

1.  **Create a New ROS 2 Package**:
    *   If you haven't already, create a new `ament_python` package in your ROS 2 workspace, named `light_controller`.
    *   Ensure `rclpy` and `std_msgs` are listed as dependencies in `package.xml`.

2.  **Develop the Command Publisher Node (`command_publisher.py`)**:
    *   Create a Python node that inherits from `rclpy.node.Node`.
    *   Initialize a publisher for a topic named `/light_command` with message type `std_msgs.msg.String`.
    *   Implement a timer callback that periodically publishes either "ON" or "OFF" commands. You can alternate them every few seconds.
    *   Include appropriate logging (`self.get_logger().info()`) to show what command is being published.

3.  **Develop the Light Subscriber Node (`light_subscriber.py`)**:
    *   Create a Python node that inherits from `rclpy.node.Node`.
    *   Initialize a subscriber for the `/light_command` topic with message type `std_msgs.msg.String`.
    *   Implement a callback function that receives the `String` message.
    *   Inside the callback, print the current state of the simulated light based on the received command (e.g., "Light is ON!" or "Light is OFF!").
    *   Include logging to show what command was received.

4.  **Create a Launch File (`light_control.launch.py`)**:
    *   Create a Python launch file within your `light_controller` package (e.g., in a `launch/` subdirectory).
    *   This launch file should start both your `command_publisher` node and your `light_subscriber` node simultaneously.
    *   Ensure both nodes are named appropriately (e.g., `command_node`, `light_node`).

## Execution and Verification

1.  **Build your Workspace**: Navigate to your ROS 2 workspace root and run `colcon build`.
2.  **Source your Workspace**: `source install/setup.bash`.
3.  **Launch the System**: `ros2 launch light_controller light_control.launch.py`.
4.  **Observe**: You should see output from both nodes in your terminal, with the `light_node` reporting changes in the light's state based on commands from the `command_node`.
5.  **Bonus - `rqt_graph`**: Run `rqt_graph` in a new terminal to visualize the nodes and topic communication in your system.

## Self-Reflection Questions

*   How would you add more light states (e.g., "DIM")?
*   What if you wanted to control multiple lights independently? How would you modify your topics?
*   What are the advantages of using separate nodes for publishing and subscribing in this scenario?

This mini-lab provides a hands-on experience in building a fundamental ROS 2 system, showcasing the power of its publish-subscribe communication pattern. Good luck!