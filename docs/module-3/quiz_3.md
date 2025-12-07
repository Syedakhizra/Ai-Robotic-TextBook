# Quiz 3: NVIDIA Isaac AI Stack Assessment

This quiz assesses your understanding of the NVIDIA Isaac AI Stack, including Isaac Sim, USD workflows, Isaac ROS perception, synthetic data generation, and Jetson deployment basics covered in Module 3.

---

### Question 1: What is the primary benefit of NVIDIA Isaac Sim for robotics development?

a) It's a lightweight physics engine for simple simulations.
b) It provides physically accurate simulation and synthetic data generation capabilities.
c) It is a tool for writing ROS 2 nodes directly in C++.
d) It's an IDE for programming Jetson devices.

### Question 2: Which technology is Isaac Sim built upon, enabling collaborative workflows and scalable composition of 3D data?

a) XML (Extensible Markup Language)
b) YAML (YAML Ain't Markup Language)
c) USD (Universal Scene Description)
d) SDF (Simulation Description Format)

### Question 3: What does the term "Domain Randomization" primarily aim to achieve when generating synthetic data for AI models?

a) To make the simulation look more realistic.
b) To reduce the computational cost of simulation.
c) To bridge the sim-to-real gap by training models on varied data.
d) To only generate data for specific, known real-world scenarios.

### Question 4: Isaac ROS leverages NVIDIA GPUs to accelerate which type of robotic tasks?

a) Hardware-in-the-loop (HIL) testing setup.
b) Physically accurate simulation of robot dynamics.
c) Real-time AI perception tasks, such as stereo depth and SLAM.
d) URDF to SDF conversion.

### Question 5: Which Isaac ROS module is designed to perform hardware-accelerated stereo disparity and depth map computation from stereo images?

a) `isaac_ros_image_pipeline`
b) `isaac_ros_unet`
c) `isaac_ros_visual_slam`
d) `isaac_ros_stereo_image_proc`

### Question 6: What is the main purpose of an NVIDIA Jetson module in a typical robotics deployment workflow with Isaac Sim?

a) To run the full Isaac Sim graphical simulation directly on the robot.
b) To serve as an edge device for real-time AI inference and processing on the robot.
c) To act as a remote development workstation for the robotics engineer.
d) To provide cloud-based GPU acceleration for simulation.

### Question 7: Which NVIDIA software stack includes the customized Ubuntu-based operating system, CUDA-X libraries, and developer tools for Jetson devices?

a) ROS 2
b) Omniverse Launcher
c) JetPack SDK
d) Isaac ROS

### Question 8: When deploying a ROS 2 Python application to a Jetson, what is the recommended step after transferring the code to the Jetson's workspace?

a) Running `sudo apt update`.
b) Directly executing the Python node.
c) Building the workspace with `colcon build`.
d) Flashing the Jetson with a new JetPack version.

### Question 9: What is the primary advantage of generating synthetic data over collecting real-world data for training robotics AI models?

a) It always produces more accurate data.
b) It provides perfect ground truth annotations automatically.
c) It requires less storage space.
d) It removes the need for any real-world testing.

### Question 10: Which type of USD feature allows combining multiple USD files non-destructively, enabling different teams to work on various aspects of a scene simultaneously?

a) Variants
b) Instancing
c) Layering (Composability)
d) Primaries

---

## Answer Key

1.  b) It provides physically accurate simulation and synthetic data generation capabilities.
2.  c) USD (Universal Scene Description)
3.  c) To bridge the sim-to-real gap by training models on varied data.
4.  c) Real-time AI perception tasks, such as stereo depth and SLAM.
5.  d) `isaac_ros_stereo_image_proc`
6.  b) To serve as an edge device for real-time AI inference and processing on the robot.
7.  c) JetPack SDK
8.  c) Building the workspace with `colcon build`.
9.  b) It provides perfect ground truth annotations automatically.
10. c) Layering (Composability)