# URDF Basics: Describing Your Robot in ROS 2

The **Unified Robot Description Format (URDF)** is an XML file format used in ROS 2 to describe all aspects of a robot. It's a powerful tool that allows you to specify the kinematic and dynamic properties of your robot, its visual appearance, and collision properties. A well-defined URDF is crucial for visualization in tools like RViz, simulation in Gazebo, motion planning, and robot control.

## What Does URDF Describe?

A URDF file primarily defines a robot's:

*   **Links**: The rigid bodies of the robot (e.g., base, arm segments, wheels). Each link has physical properties (mass, inertia), collision geometry, and visual geometry.
*   **Joints**: The connections between links, defining their relative motion. Joints can be of various types (revolute, prismatic, fixed, continuous, etc.), each with specified limits and dynamics.

### Basic URDF Structure

A URDF file starts with a `<robot>` tag, which contains one or more `<link>` and `<joint>` tags.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
  </link>

  <!-- Joint between base_link and link1 -->
  <joint name="base_to_link1_joint" type="revolute">
    <parent link="base_link" />
    <child link="link1" />
    <origin xyz="0 0 0.05" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10" velocity="10" />
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02" />
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
  </link>

  <material name="blue" />
  <material name="red" />

</robot>
```
*Description*: This URDF describes a simple two-link robot arm. It has a `base_link` and `link1` connected by a `revolute` joint. Visual properties are defined for each link.

## Key Elements in URDF

### `<link>` Element
Defines a rigid body of the robot.

*   **`<visual>`**: Describes the visual appearance of the link.
    *   `<geometry>`: Shape of the link (box, cylinder, sphere, mesh).
    *   `<material>`: Color and texture.
    *   `<origin>`: Relative position and orientation of the visual geometry.
*   **`<collision>`**: Describes the collision properties of the link. This is crucial for physics simulation and collision detection. Similar to `<visual>` but should use simpler geometries for faster computation.
*   **`<inertial>`**: Defines the mass properties (mass, inertia matrix) of the link, essential for dynamic simulation.

### `<joint>` Element
Defines the connection and motion between two links.

*   **`name`**: Unique name of the joint.
*   **`type`**: Type of motion (e.g., `revolute` for rotating about an axis, `prismatic` for linear motion, `fixed` for no motion, `continuous` for continuous rotation).
*   **`<parent link="name" />`**: Specifies the parent link.
*   **`<child link="name" />`**: Specifies the child link.
*   **`<origin xyz="x y z" rpy="roll pitch yaw" />`**: Defines the joint's position and orientation relative to the parent link.
*   **`<axis xyz="x y z" />`**: For revolute and prismatic joints, specifies the axis of rotation or translation.
*   **`<limit>`**: For revolute and prismatic joints, defines the lower and upper position limits, velocity limits, and effort limits.

## Viewing URDF in RViz

**RViz** (ROS Visualization) is a 3D visualizer for ROS 2. It's an indispensable tool for debugging and visualizing robot models described by URDF. To view a URDF model:

1.  **Ensure `robot_state_publisher` is running**: This node reads the URDF file and the joint states, then publishes the robot's state to ROS 2 topics.
2.  **Launch RViz**:
    ```bash
    rviz2
    ```
3.  **Add `RobotModel` display**: In RViz, click "Add" -> "RobotModel". In the properties for the `RobotModel` display, ensure the `Description Topic` points to the `robot_description` parameter (which is usually set by `robot_state_publisher`).

URDF provides a clear, standardized way to represent your robot's physical structure, making it possible for various ROS 2 tools to understand and interact with your robot model consistently.