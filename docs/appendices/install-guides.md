# Installation Guides (Ubuntu/ROS 2)

This appendix provides step-by-step instructions for setting up the necessary software environment to follow the textbook's content and execute the code examples.

## 1. Install Ubuntu 22.04 LTS
Follow official Ubuntu installation guides to install Ubuntu 22.04 LTS on your development machine and/or Jetson Orin device.

## 2. Install ROS 2 Humble/Iron
Refer to the official ROS 2 documentation for detailed installation instructions:
*   ROS 2 Humble: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
*   ROS 2 Iron: [https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

## 3. Install NVIDIA Drivers
Ensure the latest production branch NVIDIA drivers (version 580.65.06 or later) are installed on your development PC. Consult NVIDIA's official documentation for your specific GPU.

## 4. Install Docker & NVIDIA Container Toolkit
These are required for NVIDIA Isaac Sim and some Isaac ROS components. Follow NVIDIA's official guides for installation:
*   Docker: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
*   NVIDIA Container Toolkit: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 5. Install NVIDIA Isaac Sim
Download and install the latest stable version (5.1.0+ recommended) from the NVIDIA Developer website. Follow their installation instructions, including setting up the Omniverse Launcher:
*   NVIDIA Isaac Sim: [https://developer.nvidia.com/omniverse/download/isaac-sim](https://developer.nvidia.com/omniverse/download/isaac-sim)

## 6. Install Unity Hub & Unity 2022.3 LTS
Download Unity Hub and then install Unity 2022.3 LTS from the Unity website:
*   Unity Download: [https://unity.com/download](https://unity.com/download)

## 7. Jetson Orin Setup
For deploying applications to the Jetson Orin Nano/NX:
*   **Flash Jetson Orin**: Use NVIDIA JetPack (e.g., JetPack 6.1) to flash your Jetson Orin Nano/NX with Ubuntu 22.04.
*   **Install ROS 2 Humble on Jetson**: Follow the ROS 2 installation guide mentioned in step 2 for your Jetson device.
