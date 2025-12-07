# Example: Running Isaac ROS NITROS Stereo Depth

This example demonstrates how to set up a stereo camera in NVIDIA Isaac Sim and process its output using the hardware-accelerated `isaac_ros_stereo_image_proc` module from Isaac ROS. This module leverages NVIDIA GPUs to efficiently compute disparity and depth maps from stereo image pairs.

**Goal**: Visualize the depth output from a simulated stereo camera in Isaac Sim, processed by Isaac ROS.

## Prerequisites

*   **NVIDIA Isaac Sim**: Installed and configured (version 5.1.0+ recommended).
*   **Isaac ROS**: Workspace set up with `isaac_ros_stereo_image_proc` package built.
*   **ROS 2 Humble/Iron**: Installed and sourced.

## Example Files

This example will primarily consist of a Python launch file. Due to the interactive nature of Isaac Sim and its internal Python API for scene manipulation, the Isaac Sim portion is described conceptually, with the ROS 2 launch file handling the processing.

### `nitros_stereo_depth.py` (ROS 2 Launch File)

```python
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the Isaac ROS launch files
    isaac_ros_stereo_image_proc_dir = get_package_share_directory('isaac_ros_stereo_image_proc')

    # Composable Node Container for stereo processing
    stereo_container = ComposableNodeContainer(
        name='stereo_container',
        namespace='isaac_ros_stereo',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::StereoImageProcNode',
                name='stereo_image_proc_node',
                parameters=[{
                    'left_camera_frame': 'camera_left_link', # Replace with your robot's camera frames
                    'right_camera_frame': 'camera_right_link',
                    'disparity_depth_scaling_factor': 1.0
                }],
                remappings=[
                    # Input topics from Isaac Sim's simulated stereo camera
                    ('left/image_rect', '/stereo_camera/left/image_rect_color'),
                    ('left/camera_info', '/stereo_camera/left/camera_info'),
                    ('right/image_rect', '/stereo_camera/right/image_rect_color'),
                    ('right/camera_info', '/stereo_camera/right/camera_info'),
                    # Output topics for depth and point cloud
                    ('disparity', 'disparity_image'),
                    ('depth', 'depth_image'),
                    ('points2', 'points_xyzrgb')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        stereo_container
    ])
```

## Manual Setup in Isaac Sim (Conceptual Steps)

1.  **Launch Isaac Sim**: Start Isaac Sim from the Omniverse Launcher.
2.  **Load/Create Scene**: Load a scene with a stereo camera (e.g., from the Isaac Sim samples, or a robot model with stereo cameras).
3.  **Configure ROS 2 Bridge**: Ensure the ROS 2 bridge in Isaac Sim is active and publishing stereo camera images to topics like `/stereo_camera/left/image_rect_color`, `/stereo_camera/right/image_rect_color`, etc. (matching the remappings in the launch file above).
4.  **Visualize**: You can use RViz2 to visualize the output topics.

## Running the Example

1.  **Start Isaac Sim and your simulated robot/scene.** (As described in manual setup above).
2.  **Source your ROS 2 workspace** (where Isaac ROS packages are built).
    ```bash
    source install/setup.bash # or /opt/ros/humble/setup.bash
    ```
3.  **Launch the Isaac ROS stereo processing node**:
    ```bash
    ros2 launch examples.module-3 nitros_stereo_depth.py
    ```
4.  **Visualize with RViz2**:
    *   Open RViz2: `rviz2`
    *   Add a `PointCloud2` display and subscribe to `/isaac_ros_stereo/points_xyzrgb`.
    *   Add an `Image` display and subscribe to `/isaac_ros_stereo/depth_image`.
    *   You should see the depth map and point cloud being generated in real-time from the simulated stereo camera.

This example highlights the power of hardware acceleration in Isaac ROS for fundamental perception tasks, making it possible to achieve high-performance robotics pipelines.