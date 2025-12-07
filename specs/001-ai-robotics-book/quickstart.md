# Quickstart Guide: AI Robotics Textbook Development Environment

This guide provides instructions to quickly set up your development environment and run the code examples for the AI Robotics Textbook.

## 1. System Requirements

*   **Operating System**: Ubuntu 22.04 LTS (recommended)
*   **ROS 2 Distribution**: Humble Hawksbill or Iron Irwini
*   **Development PC**: NVIDIA GPU (RTX 4070 Ti or higher recommended) with 32-64GB RAM, running latest NVIDIA drivers.
*   **Edge Device**: NVIDIA Jetson Orin Nano/NX (8GB model recommended)
*   **Simulation Software**: Gazebo, Unity 2022.3 LTS, NVIDIA Isaac Sim 5.1.0+
*   **Optional Robot**: Unitree Go2 / OP3 / G1 (for capstone)

## 2. Environment Setup

1.  **Install Ubuntu 22.04 LTS**: Follow official Ubuntu installation guides.
2.  **Install ROS 2 Humble/Iron**:
    *   Refer to the official ROS 2 documentation: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (for Humble)
    *   Or [https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) (for Iron)
3.  **Install NVIDIA Drivers**: Ensure the latest production branch NVIDIA drivers (version 580.65.06 or later) are installed on your development PC.
4.  **Install Docker & NVIDIA Container Toolkit**: Required for Isaac Sim and some Isaac ROS components. Follow NVIDIA's official guides.
5.  **Install NVIDIA Isaac Sim**:
    *   Download and install the latest stable version (5.1.0+) from the NVIDIA Developer website: [https://developer.nvidia.com/omniverse/download/isaac-sim](https://developer.nvidia.com/omniverse/download/isaac-sim)
    *   Follow the installation instructions, including setting up the Omniverse Launcher.
6.  **Install Unity Hub & Unity 2022.3 LTS**: Download from the Unity website: [https://unity.com/download](https://unity.com/download)
7.  **Jetson Orin Setup**:
    *   Flash your Jetson Orin Nano/NX with JetPack 6.1 (Ubuntu 22.04).
    *   Install ROS 2 Humble on the Jetson.

## 3. Clone the Repository

```bash
git clone https://github.com/your-organization/ai-robotics-textbook.git
cd ai-robotics-textbook
```

## 4. Run Code Examples

Each module's code examples will be located in the `examples/module-X/` directory.

To run a typical Python ROS 2 example:

```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash # If you have a local workspace

# Navigate to the example directory
cd examples/module-1/example_name

# Run the Python script
python3 your_example_node.py
```

For simulation examples in Gazebo, Unity, or Isaac Sim, specific launch commands or project loading instructions will be provided within the respective module's text.

## 5. Build and Serve Docusaurus Site

To view the textbook content locally:

```bash
# Install Node.js and npm (if not already installed)
# Follow official Node.js installation guides

# Install Docusaurus dependencies
npm install

# Start local development server
npm start
```

This will open the site in your web browser, typically at `http://localhost:3000`. Changes to Markdown files will automatically reload the site.
