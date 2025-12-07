# ROS 2 Launch Files: Orchestrating Multiple Nodes

In real-world robotic systems, you rarely run a single ROS 2 node in isolation. Instead, you typically need to start multiple nodes simultaneously, often with specific configurations, remappings, and conditional logic. **Launch files** in ROS 2 provide a powerful and flexible way to define and manage the startup choreography of your entire robotic application.

## What are Launch Files?

A launch file is a Python script (or XML, though Python is now preferred for its flexibility) that describes how to start and configure a set of ROS 2 nodes and other processes. They are used for:

*   **Starting Multiple Nodes**: Launching all necessary components of a robot system (e.g., sensor drivers, navigation stack, control algorithms).
*   **Parameter Configuration**: Setting initial parameters for nodes.
*   **Remapping**: Changing the names of topics, services, or actions used by nodes without modifying their source code.
*   **Conditional Logic**: Starting nodes or groups of nodes only if certain conditions are met (e.g., if a specific sensor is present).
*   **Group Management**: Organizing nodes into logical groups for easier management and logging.

## Basic Structure of a Python Launch File

ROS 2 launch files are typically Python scripts that return a `LaunchDescription` object. This object contains a list of launch actions to be performed.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_node'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_node',
            prefix='xterm -e', # Open in new terminal
            output='screen'
        )
    ])
```
*Description*: This simple launch file starts two nodes from the `turtlesim` package: `turtlesim_node` (the simulator) and `turtle_teleop_key` (for keyboard control). The `prefix` and `output` options for `teleop_node` ensure it opens in a new terminal window, making it interactive.

## Key Launch Actions

The `launch` and `launch_ros` packages provide various actions to define behaviors within a launch file:

*   **`Node`**: Represents a ROS 2 executable.
    *   `package`: Name of the ROS 2 package containing the executable.
    *   `executable`: Name of the executable to run.
    *   `name`: Assigns a specific name to the node (overriding the default).
    *   `namespace`: Places the node within a ROS 2 namespace.
    *   `parameters`: Provides parameters to the node.
    *   `remappings`: Changes topic/service/action names.
    *   `arguments`: Passes command-line arguments to the executable.
*   **`ExecuteProcess`**: Runs any arbitrary command or executable (not necessarily a ROS 2 node).
*   **`IncludeLaunchDescription`**: Includes another launch file, allowing for modular launch configurations.
*   **`GroupAction`**: Groups multiple actions together, often used with `PushRosNamespace` to apply a namespace to an entire group.
*   **`SetParameter`**: Sets a global ROS 2 parameter.
*   **`OpaqueFunction`**: Allows arbitrary Python code to be executed within the launch file.

## Running a Launch File

To run a launch file, navigate to the root of your ROS 2 workspace (if your launch file is in a package within that workspace) and use the `ros2 launch` command:

```bash
# Navigate to your workspace (e.g., ~/ros2_ws)
cd ~/ros2_ws

# Source your workspace (if not already sourced)
source install/setup.bash

# Launch the file
ros2 launch my_package my_launch_file.py
```
*Description*: This command assumes your launch file `my_launch_file.py` is located in the `launch` directory of `my_package`.

## Why Use Launch Files?

*   **Reproducibility**: Ensures that an entire robotic system can be started in a consistent manner every time.
*   **Ease of Use**: Simplifies the process of starting complex systems for users who may not be familiar with every individual component.
*   **Flexibility**: Allows for easy modification of node configurations and interconnections without touching node source code.
*   **Debugging**: Provides structured output and logging, aiding in debugging multi-node systems.

Launch files are an indispensable tool for any serious ROS 2 developer, enabling the robust and flexible deployment of robotic applications.