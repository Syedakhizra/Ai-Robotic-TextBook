# Module 1: ROS 2 Foundations

This module introduces the fundamental concepts of the Robot Operating System 2 (ROS 2), providing a solid foundation for developing robotic applications. We will explore the core components of ROS 2, learn how to create and manage packages, and understand how to describe robot structures using URDF.

## Module Objectives

Upon completion of this module, you will be able to:

*   Understand the ROS 2 communication mechanisms: nodes, topics, services, and actions.
*   Create and build ROS 2 packages using `rclpy` (Python client library).
*   Launch multiple ROS 2 nodes using launch files.
*   Describe the kinematics and visual aspects of a robot using URDF.
*   Execute basic ROS 2 applications and interpret their behavior.

## Prerequisites

To make the most of this module, you should have:

*   Basic proficiency in Python programming.
*   Familiarity with the Linux command line (especially Ubuntu 22.04 LTS).
*   A working ROS 2 Humble/Iron installation as per the [Installation Guides](../appendices/install-guides.md).

## Module Sections

1.  [Introduction to ROS 2](introduction.md)
2.  [ROS 2 Concepts: Nodes, Topics, Publishers, Subscribers](concepts.md)
3.  [ROS 2 Services and Actions](services_actions.md)
4.  [Packages and Workspaces](packages_workspaces.md)
5.  [Launch Files](launch_files.md)
6.  [URDF Basics](urdf_basics.md)
7.  [Mini-Lab 1: Build a Multi-Node ROS 2 System](mini_lab_1.md)
8.  [Quiz 1: ROS 2 Foundations Assessment](quiz_1.md)

## Code Examples

Throughout this module, you will find practical `rclpy` code examples to illustrate ROS 2 concepts:

*   **Simple Publisher/Subscriber**: Demonstrates basic topic communication.
*   **ROS 2 Service Client/Server**: Shows how to implement request-response patterns.
*   **Displaying a Simple URDF Model**: Visualizes a robot description in RViz.
