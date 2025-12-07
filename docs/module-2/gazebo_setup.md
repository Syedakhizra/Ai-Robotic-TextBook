# Gazebo Setup and Physics: Building Your Simulated World

Gazebo is a powerful 3D robotics simulator that provides the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It offers a robust physics engine, high-quality graphics, and convenient programmatic interfaces.

## 1. Gazebo Installation

Gazebo is typically installed alongside ROS 2. If you followed the ROS 2 installation guide in the appendices, you likely already have it. To ensure you have the full Gazebo (Fortress or Harmonic, depending on your ROS 2 version) with ROS 2 integration, you can verify with:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs # For ROS 2 Humble
```
*Description*: This command ensures all necessary Gazebo packages for ROS 2 Humble are installed.

## 2. Launching Gazebo

You can launch Gazebo in two main ways:

*   **Stand-alone GUI**:
    ```bash
    gazebo # Launches the GUI and an empty world
    ```
*   **With ROS 2 Launch File**: More commonly, Gazebo is launched as part of a ROS 2 launch file to bring up a specific world and robots.
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def generate_launch_description():
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
        
        # Launch Gazebo with an empty world
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'gazebo_model_path': os.path.join(get_package_share_directory('my_robot_description'), 'models')}.items()
        )

        return LaunchDescription([
            gazebo_launch
        ])
    ```
    *Description*: This Python launch file starts Gazebo with the default empty world. The `gazebo_model_path` argument can be used to add custom models.

## 3. Gazebo Physics Engine

Gazebo relies on a physics engine to simulate realistic interactions between objects. By default, Gazebo uses **ODE (Open Dynamics Engine)**, but it also supports others like Bullet, DART, and Simbody. The physics engine handles:

*   **Gravity**: Applying gravitational forces to all simulated objects.
*   **Collisions**: Detecting and resolving physical contact between objects.
*   **Friction**: Modeling the resistance to motion between surfaces.
*   **Joint Dynamics**: Simulating forces and torques on robot joints.
*   **Inertia**: Representing how objects resist changes in motion based on their mass distribution.

### Physics Parameters

You can configure physics parameters in your Gazebo world files (typically `.world` XML files). Key parameters include:

*   **`real_time_update_rate`**: How many physics updates to perform per real-time second.
*   **`max_step_size`**: The maximum time step for the physics engine. Smaller values lead to more accurate but slower simulations.
*   **`gravity`**: Vector defining the direction and magnitude of gravity.
*   **`iters`**: Number of iterations for the solver (higher = more stable, slower).

### Example: Simple Pendulum

To demonstrate basic physics, we can simulate a simple pendulum. This involves defining a world with a fixed base, a freely rotating joint, and a mass attached to it. The motion of the pendulum will be governed by Gazebo's physics engine.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
          <friction_model>cone_model</friction_model>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
        </constraints>
        <contact>
          <ode>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>1e+08</kp>
            <kd>1</kd>
          </ode>
        </contact>
      </ode>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <include>
      <uri>model://sun</uri>
    </include>

    <model name="base_link">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link_fixed">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx> <iyy>0.0001</iyy> <izz>0.0001</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
            <specular>0.01 0.01 0.01 1</specular>
            <emissive>0 0 0 0</emissive>
          </material>
        </visual>
        <collision>
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
        </collision>
      </link>

      <joint name="pendulum_joint" type="revolute">
        <parent>base_link_fixed</parent>
        <child>pendulum_link</child>
        <pose>0 0 -0.1 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>

      <link name="pendulum_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx> <iyy>0.083</iyy> <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry>
            <box>
              <size>0.02 0.02 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 1 1</specular>
            <emissive>0 0 0 0</emissive>
          </material>
        </visual>
        <collision>
          <geometry>
            <box>
              <size>0.02 0.02 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```
*Description*: This SDF file defines a simple world with a fixed base and a pendulum that swings under gravity. You can save this as `simple_pendulum.world` and launch Gazebo with `gazebo simple_pendulum.world`.

By understanding Gazebo's setup and physics configuration, you can create realistic and robust simulation environments for your robotic applications.