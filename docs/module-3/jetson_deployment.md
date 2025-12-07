# Jetson Deployment Basics: Running Robotics Applications on Edge Devices

**NVIDIA Jetson** modules are powerful, energy-efficient AI supercomputers designed for edge devices. They are ideal for deploying robotics applications that require real-time AI inference and processing directly on the robot. This section will cover the fundamental concepts and steps involved in deploying your ROS 2 and Isaac ROS applications to a Jetson Orin device.

## 1. Why Jetson for Robotics Deployment?

Jetson modules offer several key advantages for robotics:

*   **Edge AI Performance**: High-performance GPUs enable real-time AI inference, crucial for perception, navigation, and control tasks.
*   **Power Efficiency**: Designed for low-power operation, making them suitable for battery-powered robots.
*   **Small Form Factor**: Compact size allows for integration into a wide range of robotic platforms.
*   **Integrated Hardware**: Includes CPU, GPU, and specialized AI accelerators (like the NVDLA) on a single module.
*   **ROS 2 and Isaac ROS Support**: Deep integration with the ROS 2 ecosystem and NVIDIA's Isaac ROS hardware-accelerated packages.

## 2. Jetson Software Stack

The Jetson runs on **JetPack SDK**, NVIDIA's comprehensive software stack that includes:

*   **Linux for Tegra (L4T)**: A customized Ubuntu-based operating system.
*   **CUDA-X Accelerated Libraries**: CUDA Toolkit, cuDNN, TensorRT for GPU-accelerated computing.
*   **Developer Tools**: Debuggers, profilers, and sample applications.

## 3. Basic Deployment Workflow

The general workflow for deploying a ROS 2 application to a Jetson involves:

1.  **Develop on Workstation**: Develop and simulate your ROS 2 application on your powerful development PC (Ubuntu 22.04 + ROS 2 Humble).
2.  **Cross-Compilation (if C++)**: For C++ ROS 2 packages, you might need to cross-compile for the Jetson's ARM architecture. However, often a native build on the Jetson is sufficient for Python packages and smaller C++ projects.
3.  **Transfer Code**: Copy your ROS 2 workspace (containing your packages) to the Jetson device.
4.  **Build on Jetson**: Build the ROS 2 packages directly on the Jetson.
5.  **Run on Jetson**: Execute your ROS 2 nodes or launch files on the Jetson.

## 4. Setting up a ROS 2 Workspace on Jetson

Assuming you have flashed JetPack onto your Jetson Orin (which includes Ubuntu 22.04) and installed ROS 2 Humble as per the [Installation Guides](../appendices/install-guides.md):

1.  **Create a ROS 2 workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Copy your ROS 2 package(s)**: Transfer your developed ROS 2 packages from your development PC to the `~/ros2_ws/src` directory on the Jetson. You can use `scp` or `rsync`.
    ```bash
    # From your development PC
    scp -r /path/to/my_ros2_package_folder username@jetson_ip_address:~/ros2_ws/src/
    ```
3.  **Install dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  **Build your workspace**:
    ```bash
    colcon build --symlink-install
    ```
5.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## 5. Running ROS 2 Nodes on Jetson

Once your workspace is built and sourced, you can run your ROS 2 nodes on the Jetson just like on your development PC:

```bash
# Example: Run a Python node
ros2 run my_package my_python_node_executable

# Example: Launch a Python launch file
ros2 launch my_package my_launch_file.py
```

## 6. Utilizing Isaac ROS on Jetson

For maximum performance, particularly for perception tasks, leverage Isaac ROS. Ensure you have built the necessary Isaac ROS packages in your Jetson's ROS 2 workspace.

```bash
# Example: Running an Isaac ROS perception node (conceptual)
# Requires Isaac ROS packages to be built in your workspace
ros2 launch isaac_ros_stereo_image_proc stereo_image_proc.launch.py # Example from previous section
```

By following these deployment basics, you can effectively transfer your robotics applications from the development environment to the powerful NVIDIA Jetson edge device, enabling your robots to execute complex AI tasks in real-time.