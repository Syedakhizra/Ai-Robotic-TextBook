# Hardware Requirements

This appendix details the recommended and optional hardware for effectively using this textbook, especially for reproducing code examples and capstone projects.

## 1. Development PC

*   **Operating System**: Ubuntu 22.04 LTS (recommended for best compatibility with ROS 2 and NVIDIA software).
*   **Processor**: Modern multi-core CPU (e.g., Intel Core i7/i9, AMD Ryzen 7/9 or equivalent).
*   **RAM**: 32-64GB RAM (32GB minimum, 64GB recommended for large simulations and AI models).
*   **GPU**: NVIDIA GPU (RTX 4070 Ti or higher recommended).
    *   **Drivers**: Latest production branch NVIDIA drivers (version 580.65.06 or later).
    *   **VRAM**: 12GB+ VRAM recommended for NVIDIA Isaac Sim.

## 2. Edge Device (for physical robot control/deployment)

*   **Device**: NVIDIA Jetson Orin Nano/NX (8GB model recommended).
*   **Operating System**: JetPack 6.1 (Ubuntu 22.04).
*   **Sensors**:
    *   RealSense D455 Depth Camera (or similar depth sensor).
    *   IMU (Inertial Measurement Unit).

## 3. Optional Robot (for Capstone Project)

*   **Models**: Unitree Go2 / OP3 / G1 (or any similar small to medium-sized humanoid or quadruped robot capable of ROS 2 integration).
*   **Requirements**: The robot should ideally support:
    *   ROS 2 Humble/Iron integration.
    *   Sufficient payload capacity for sensors.
    *   Programmable control interface.

## 4. Software Environment (brief overview, detailed in Install Guides)

*   ROS 2 Humble or Iron.
*   Gazebo (typically installed with ROS 2).
*   Unity 2022.3 LTS.
*   NVIDIA Isaac Sim 5.1.0+.
*   Docker & NVIDIA Container Toolkit.
