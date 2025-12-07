# Module 3: NVIDIA Isaac AI Stack

This module explores the NVIDIA Isaac AI Stack, a comprehensive platform designed for the development, simulation, and deployment of AI-powered robotics applications. We will dive into Isaac Sim for high-fidelity simulation and synthetic data generation, and Isaac ROS for hardware-accelerated robot perception and processing on NVIDIA Jetson platforms.

## Module Objectives

Upon completion of this module, you will be able to:

*   Understand the capabilities of NVIDIA Isaac Sim and its Omniverse USD workflow.
*   Utilize Isaac ROS modules for accelerated perception tasks like stereo depth and SLAM.
*   Generate synthetic data and apply domain randomization techniques for training AI models.
*   Deploy ROS 2 applications to NVIDIA Jetson Orin devices.
*   Integrate Isaac Sim with ROS 2 for advanced simulation and development.

## Prerequisites

To make the most of this module, you should have:

*   A solid understanding of ROS 2 fundamentals (as covered in Module 1).
*   Familiarity with robotics simulation concepts (as covered in Module 2).
*   Basic understanding of AI/Machine Learning concepts.
*   A working installation of NVIDIA Isaac Sim 5.1.0+ and an NVIDIA Jetson Orin device (or access to one) as per the [Installation Guides](../appendices/install-guides.md).

## Module Sections

1.  [Introduction to NVIDIA Isaac Sim](isaac_sim_intro.md)
2.  [USD Workflow: Universal Scene Description](usd_workflow.md)
3.  [Isaac ROS Perception: Accelerated Perception Modules](isaac_ros_perception.md)
4.  [Isaac ROS SLAM: Simultaneous Localization and Mapping](isaac_ros_slam.md)
5.  [Synthetic Data Generation and Domain Randomization](synthetic_data.md)
6.  [Jetson Deployment Basics](jetson_deployment.md)
7.  [Mini-Lab 3: Generate Synthetic Data and Train an Object Detector](mini_lab_3.md)
8.  [Quiz 3: NVIDIA Isaac AI Stack Assessment](quiz_3.md)

## Code Examples

Throughout this module, you will find practical code examples for the NVIDIA Isaac AI Stack:

*   **Running Isaac ROS NITROS Stereo Depth**: Demonstrates accelerated perception.
*   **Domain Randomization for Object Pose Estimation**: Illustrates synthetic data generation.
*   **Deploying a Simple ROS 2 Node to Jetson Orin**: Shows how to deploy applications to edge devices.
