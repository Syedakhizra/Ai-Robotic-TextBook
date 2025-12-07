# Unity for Human-Robot Interaction (HRI): Engaging with Virtual Robots

While Gazebo excels in physics-accurate simulation for industrial and research robotics, **Unity** offers unparalleled graphical fidelity, user interface development tools, and interactive capabilities. This makes Unity an increasingly popular platform for **Human-Robot Interaction (HRI)** research and development, especially for scenarios requiring rich visualization, intuitive control interfaces, and sophisticated human-robot collaboration.

## 1. Why Unity for HRI?

Unity, originally a game engine, brings several advantages to HRI:

*   **High-Fidelity Graphics**: Create visually stunning and realistic robot environments that enhance user immersion and understanding.
*   **Intuitive UI Development**: Robust UI system (Unity UI or UI Toolkit) allows for the rapid development of custom control panels, data displays, and interactive dashboards for human operators.
*   **Cross-Platform Deployment**: Easily deploy HRI applications to desktop, mobile, web, and even AR/VR platforms, enabling diverse interaction modalities.
*   **Rich Asset Ecosystem**: Access to Unity Asset Store for pre-built 3D models, environments, and tools accelerates development.
*   **ROS Integration**: Unity Robotics Hub and `ros2-for-unity` packages provide powerful bridges for seamless communication with ROS 2, allowing Unity simulations to act as ROS 2 nodes.
*   **Complex Interactions**: The event-driven and object-oriented nature of Unity (C# scripting) is well-suited for modeling complex human-robot interaction logic.

## 2. Setting up a Unity Project for Robotics

1.  **Install Unity Hub and Unity 2022.3 LTS**: As covered in the [Installation Guides](../appendices/install-guides.md).
2.  **Create a New Unity Project**: Open Unity Hub, create a new 3D project.
3.  **Install Unity Robotics Packages**:
    *   **Unity Robotics Hub**: This is a meta-package providing a collection of tools, tutorials, and examples for robotics in Unity. Install it via the Unity Package Manager (Window > Package Manager > Unity Registry > Robotics).
    *   **ROS-TCP-Connector**: This package facilitates communication between Unity and ROS 2 over TCP. Install it via the Unity Package Manager (it's often a dependency of other Robotics Hub packages).
4.  **Import Robot Models**: Import your robot's 3D model (e.g., FBX, URDF-converted meshes) into Unity. You might use the `URDF-Importer` package from the Robotics Hub to bring in URDF models.

## 3. Basic HRI in Unity: A Simple Control Interface

Let's consider a simple example: controlling a simulated robot's movement using a UI joystick or buttons.

### Conceptual Steps

1.  **Robot Model**: Import a simple robot model into your Unity scene.
2.  **ROS 2 Connection**: Set up the `ROS-TCP-Connector` to establish communication with a ROS 2 system.
3.  **ROS Publisher**: In a C# script attached to your robot or a dedicated GameObject, create a `RosPublisher` for `geometry_msgs/msg/Twist` messages (for velocity commands).
4.  **UI Elements**: Create UI buttons (e.g., "Forward", "Left", "Right") or a virtual joystick using Unity UI.
5.  **Interaction Script**: Write a C# script to handle UI interactions. When a button is pressed or joystick moved, generate a `Twist` message and publish it via the `RosPublisher`.
6.  **ROS 2 Subscriber (External)**: In your ROS 2 environment (e.g., a Python `rclpy` node), create a subscriber to the `cmd_vel` topic that receives these `Twist` messages and converts them into appropriate commands for the robot model in Unity (if the robot's physics are handled by Unity).

### Example: Publishing `Twist` Messages from Unity (C#)

```csharp
using RosMessageTypes.Geometry; // For TwistMsg
using RosMessageTypes.Std; // For HeaderMsg if needed
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI; // For UI elements

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    public GameObject rosConnector; // Assign ROSConnection GameObject in Inspector
    public Button forwardButton;
    public Button stopButton;

    // The speed we want to move the robot
    public float linearSpeed = 0.5f; // meters/second
    public float angularSpeed = 0.5f; // radians/second

    void Start()
    {
        ros = ROSConnection.Get  Instance();
        if (ros == null && rosConnector != null)
        {
            ros = rosConnector.GetComponent<ROSConnection>();
        }
        if (ros != null)
        {
            ros.RegisterPublisher<TwistMsg>(topicName);
        }
        else
        {
            Debug.LogError("ROSConnection not found or assigned!");
        }

        forwardButton.onClick.AddListener(MoveForward);
        stopButton.onClick.AddListener(StopRobot);
    }

    public void MoveForward()
    {
        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(linearSpeed, 0, 0),
            angular = new Vector3Msg(0, 0, 0)
        };
        ros.Publish(topicName, twist);
        Debug.Log("Published Twist: Move Forward");
    }

    public void StopRobot()
    {
        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg(0, 0, 0),
            angular = new Vector3Msg(0, 0, 0)
        };
        ros.Publish(topicName, twist);
        Debug.Log("Published Twist: Stop");
    }
}
```
*Description*: This C# script in Unity publishes `TwistMsg` messages to the `/cmd_vel` ROS 2 topic when UI buttons are clicked.

By leveraging Unity's robust environment, developers can create highly interactive and visually engaging HRI simulations, providing a more intuitive way for humans to interact with and understand robotic systems.