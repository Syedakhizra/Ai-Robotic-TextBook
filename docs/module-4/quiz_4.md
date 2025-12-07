# Quiz 4: Vision-Language-Action (VLA) Assessment

This quiz assesses your understanding of Vision-Language-Action models in robotics, including voice-to-text, LLM planning, perception pipelines, motion planning, ROS integration, and safety.

---

### Question 1: What is the primary function of a Vision-Language Model (VLM) in robotics?

a) To translate spoken commands directly into motor signals without any intermediate processing.
b) To process and relate information from visual data and natural language for robot action.
c) To generate realistic 3D environments for robot simulation.
d) To optimize robot hardware design for better human interaction.

### Question 2: Which OpenAI model is highly effective for converting spoken human commands into text for a VLA system?

a) GPT-3
b) DALL-E
c) Whisper
d) CLIP

### Question 3: What capability do Large Language Models (LLMs) bring to robot task planning that is difficult for traditional planning methods?

a) Precise inverse kinematics calculations.
b) Interpreting ambiguous natural language commands and decomposing high-level goals.
c) Direct real-time control of robot joint velocities.
d) Fast collision detection within complex environments.

### Question 4: In a perception pipeline, what information does "pose estimation" primarily provide that "object detection" alone does not?

a) The class label of the object.
b) The 2D bounding box of the object in an image.
c) The 3D position and orientation (6-DoF) of the object in the robot's frame.
d) The color histogram of the object.

### Question 5: Which ROS 2 package is most commonly used for complex motion planning, collision avoidance, and robot control interfaces?

a) `rclpy`
b) `turtlesim`
c) `MoveIt 2`
d) `robot_state_publisher`

### Question 6: What is the main purpose of "grounding" an LLM's understanding in the robot's perception?

a) To prevent the LLM from generating any output.
b) To ensure the LLM's language understanding relates to the physical world the robot perceives.
c) To speed up the LLM's response time.
d) To replace the need for motion planning algorithms.

### Question 7: When a robot is operating in close proximity to a human, what is a crucial safety mechanism implemented in Human-Robot Collaboration (HRC)?

a) Completely disabling all robot movement.
b) Continuously monitoring the human's distance and adjusting robot speed or stopping.
c) Increasing the robot's operational speed to quickly complete tasks.
d) Relying solely on the human to avoid the robot.

### Question 8: In a ROS 2 VLA pipeline, if the LLM determines a high-level task like "navigate to the kitchen," which ROS 2 communication pattern would be most appropriate for this long-running, goal-oriented task with continuous feedback?

a) Topics
b) Services
c) Actions
d) Parameters

### Question 9: Why is "fail-safe design" a critical principle in robotics safety?

a) It makes robots move faster.
b) It ensures systems fail in a manner that prevents harm.
c) It reduces the cost of robot maintenance.
d) It allows robots to self-repair after a failure.

### Question 10: Which of these is an example of an unsafe or ambiguous command that a VLA robot should ideally flag or seek clarification for?

a) "Pick up the blue box."
b) "Go to the charging station."
c) "Put the tool next to the robot." (If multiple tools or robots are present)
d) "Report current battery status."

---

## Answer Key

1.  b) To process and relate information from visual data and natural language for robot action.
2.  c) Whisper
3.  b) Interpreting ambiguous natural language commands and decomposing high-level goals.
4.  c) The 3D position and orientation (6-DoF) of the object in the robot's frame.
5.  c) `MoveIt 2`
6.  b) To ensure the LLM's language understanding relates to the physical world the robot perceives.
7.  b) Continuously monitoring the human's distance and adjusting robot speed or stopping.
8.  c) Actions
9.  b) It ensures systems fail in a manner that prevents harm.
10. c) "Put the tool next to the robot." (If multiple tools or robots are present)