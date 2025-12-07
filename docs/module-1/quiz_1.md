# Quiz 1: ROS 2 Foundations Assessment

This quiz is designed to test your understanding of the core concepts covered in Module 1: ROS 2 Foundations. Choose the best answer for each question.

---

### Question 1: What is the primary purpose of a ROS 2 Node?

a) To define the communication protocols for the entire robot system.
b) To act as an executable process that performs a specific, isolated task.
c) To store global parameters for all other ROS 2 components.
d) To visualize robot sensor data in a 3D environment.

### Question 2: Which ROS 2 communication mechanism is best suited for continuous, asynchronous data streams, like sensor readings?

a) Services
b) Actions
c) Topics
d) Parameters

### Question 3: In the `rclpy` code for a publisher, what does the `queue_size` argument (e.g., `self.create_publisher(String, 'topic', 10)`) typically refer to?

a) The number of subscribers that can connect to the topic.
b) The maximum number of messages to store if subscribers are slow.
c) The publishing rate in Hertz.
d) The priority of the published messages.

### Question 4: You want to send a request to a ROS 2 component and wait for a single, immediate response. Which communication mechanism should you use?

a) Topic
b) Service
c) Action
d) Parameter

### Question 5: What is the main benefit of using ROS 2 Launch Files?

a) To write individual ROS 2 nodes in Python.
b) To define custom message types for ROS 2 communication.
c) To orchestrate the startup and configuration of multiple ROS 2 nodes simultaneously.
d) To automatically generate URDF files for a robot.

### Question 6: What does URDF stand for?

a) Universal Robot Data Format
b) Unified Robot Description File
c) Unified Robot Description Format
d) Universal Robotic Device File

### Question 7: Which URDF element is used to define the physical properties (mass, inertia) of a robot's rigid body?

a) `<visual>`
b) `<collision>`
c) `<link>` (specifically, within its `<inertial>` sub-element)
d) `<joint>`

### Question 8: A joint type that allows only rotation around a single axis, with defined angular limits, is called a:

a) Prismatic joint
b) Fixed joint
c) Continuous joint
d) Revolute joint

### Question 9: Which `colcon` command is used to build packages in a ROS 2 workspace?

a) `colcon install`
b) `colcon run`
c) `colcon build`
d) `colcon source`

### Question 10: After building a new ROS 2 package in your workspace, what must you do before you can run its executables in your current terminal session?

a) Restart your computer.
b) Run `ros2 install <package_name>`.
c) Source your workspace's `install/setup.bash` (or equivalent).
d) Edit the package's `CMakeLists.txt` file.

---

## Answer Key

1.  b) To act as an executable process that performs a specific, isolated task.
2.  c) Topics
3.  b) The maximum number of messages to store if subscribers are slow.
4.  b) Service
5.  c) To orchestrate the startup and configuration of multiple ROS 2 nodes simultaneously.
6.  c) Unified Robot Description Format
7.  c) `<link>` (specifically, within its `<inertial>` sub-element)
8.  d) Revolute joint
9.  c) `colcon build`
10. c) Source your workspace's `install/setup.bash` (or equivalent).