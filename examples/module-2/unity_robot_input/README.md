# Basic Unity Scene with a Robot and User Input

This directory will contain a Unity project or UnityPackage (`.unitypackage`) demonstrating a basic human-robot interaction (HRI) scenario. The goal is to provide a simple Unity scene where a virtual robot can be controlled via user input (e.g., UI buttons) and communicate with a ROS 2 system.

## Purpose

*   Illustrate how to set up a Unity project for robotics.
*   Demonstrate basic ROS 2 integration using `ROS-TCP-Connector`.
*   Show how to create simple UI elements for robot control.

## Content (to be developed manually in Unity Editor)

This example will require manual creation within the Unity Editor. It should include:

1.  **Unity Project**: A Unity project configured with the Unity Robotics Hub and ROS-TCP-Connector packages.
2.  **Robot Model**: A simple 3D robot model (e.g., a differential drive robot or a basic manipulator).
3.  **ROS 2 Publisher**: A C# script to publish `geometry_msgs/msg/Twist` commands to a ROS 2 topic (e.g., `/cmd_vel`) based on user input.
4.  **UI Elements**: Simple UI buttons (e.g., Forward, Backward, Turn Left, Turn Right) to trigger robot movement commands.
5.  **Scene Setup**: A basic 3D scene with the robot, ground plane, and UI canvas.

## Manual Steps to Create/Import

1.  **Open Unity Editor**: Launch Unity Hub and open a new 3D project.
2.  **Install Robotics Packages**: Go to Window > Package Manager > Unity Registry. Install "Unity Robotics Hub" and "ROS-TCP-Connector".
3.  **Import Robot**: Import a simple robot model or create a primitive one.
4.  **Create UI**: Use GameObject > UI > Canvas to create a UI for buttons.
5.  **Develop C# Script**: Implement the `RobotController` script as outlined in the textbook, attaching it to an appropriate GameObject.
6.  **Export (Optional)**: If distributing as a `.unitypackage`, go to Assets > Export Package.

## Example Use Case

Once created, you should be able to:

1.  Run a ROS 2 listener for `/cmd_vel` (e.g., `ros2 topic echo /cmd_vel`).
2.  Run the Unity scene.
3.  Click the UI buttons in Unity, and observe `Twist` messages being published on the ROS 2 topic.
