# Capstone Project: Autonomous Humanoid - Evaluation Rubric

This rubric will be used to assess the successful completion and performance of the Autonomous Humanoid Capstone Project. It covers various aspects of the VLA system, from functionality and integration to safety and documentation.

## 1. System Functionality (40%)

| Criteria                                  | Not Met (0)                               | Partially Met (1-3)                       | Met (4-6)                                     | Exceeds (7-8)                                      | Score |
| :---------------------------------------- | :---------------------------------------- | :---------------------------------------- | :-------------------------------------------- | :------------------------------------------------- | :---- |
| **Voice Command Processing**              | Cannot process any voice commands.        | Processes limited, exact voice commands.  | Processes most common voice commands.         | Handles variations, ambiguities, and asks clarification. |       |
| **Object Perception (Detection & Pose)**  | Cannot detect or localize objects.        | Detects some objects; pose estimation is inaccurate. | Reliably detects and provides accurate 6-DoF poses for target objects. | Robust to varying conditions; detects novel objects (conceptual). |       |
| **LLM-based Task Planning**               | Cannot generate any action plans.         | Generates simple, inflexible action plans. | Decomposes high-level commands into correct action sequences. | Exhibits commonsense reasoning; handles unexpected situations gracefully. |       |
| **Motion Planning & Execution**           | Fails to plan or execute motions.         | Plans/executes some motions, but with collisions or errors. | Plans and executes collision-free motions for defined tasks. | Optimizes paths (time/energy); robust to minor dynamic obstacles. |       |
| **End-to-End Task Completion**            | Cannot complete any end-to-end tasks.     | Completes simple end-to-end tasks inconsistently. | Consistently completes defined end-to-end tasks (e.g., pick-and-place). | Completes complex multi-step tasks; recovers from minor failures. |       |

## 2. Integration & Modularity (30%)

| Criteria                                  | Not Met (0)                               | Partially Met (1-2)                       | Met (3-4)                                     | Exceeds (5-6)                                      | Score |
| :---------------------------------------- | :---------------------------------------- | :---------------------------------------- | :-------------------------------------------- | :------------------------------------------------- | :---- |
| **ROS 2 Integration**                     | Components not integrated with ROS 2.     | Basic ROS 2 communication, but messy topic usage. | Uses ROS 2 topics/services/actions effectively for component communication. | Well-structured ROS 2 graph; uses appropriate communication patterns; extensible. |       |
| **Simulation Environment Usage**          | No simulation environment used.           | Basic object spawning; no complex interactions. | Effectively uses Gazebo/Isaac Sim for realistic robot and environment modeling. | Leverages advanced features (e.g., sensor noise, dynamic elements, DR for testing). |       |
| **Code Modularity & Readability**         | Monolithic code; hard to understand.      | Some modularity, but inconsistent.        | Code is modular, well-commented, and follows Python/ROS 2 best practices. | Excellent modularity; reusable components; clear separation of concerns. |       |
| **Reproducibility of Examples**           | Examples cannot be reproduced.            | Examples require significant manual setup. | All code examples are clearly documented and reproducible with provided instructions. | Highly reproducible; Docker/containerized setup for environment. |       |
| **Documentation of Design Choices**       | No design documentation.                  | Minimal design notes.                     | Clear documentation of system design and architectural decisions. | Comprehensive design documents (e.g., `system_design.md` completed). |       |

## 3. Safety & Robustness (20%)

| Criteria                                  | Not Met (0)                               | Partially Met (1-2)                       | Met (3-4)                                     | Exceeds (5-6)                                      | Score |
| :---------------------------------------- | :---------------------------------------- | :---------------------------------------- | :-------------------------------------------- | :------------------------------------------------- | :---- |
| **Collision Avoidance**                   | Robot collides frequently.                | Avoids some collisions, but not reliably. | Plans collision-free paths for static and known dynamic obstacles. | Proactively avoids humans; handles unexpected dynamic obstacles gracefully. |       |
| **Error Handling & Feedback**             | System crashes on errors; no feedback.    | Basic error messages; limited feedback.   | Provides clear feedback to human on task status and asks for clarification on ambiguity. | Robust error recovery; intelligent clarification; self-diagnoses simple issues. |       |
| **Adherence to Safety Principles**        | Ignores safety principles.                | Some safety measures implemented.         | Demonstrates awareness and implementation of fundamental safety principles (e.g., E-Stop, speed limits). | Incorporates advanced safety protocols; proactive human-robot collaboration safety. |       |

## 4. Presentation & Final Report (10%)

| Criteria                      | Not Met (0)                               | Partially Met (1)                       | Met (2)                                     | Exceeds (3)                                       | Score |
| :---------------------------- | :---------------------------------------- | :-------------------------------------- | :------------------------------------------ | :------------------------------------------------ | :---- |
| **Demonstration Clarity**     | Confusing or incomplete demonstration.    | Demonstrates limited functionality.     | Clear demonstration of core functionalities. | Engaging and informative; highlights key technical achievements. |       |
| **Report Completeness**       | Missing major sections.                   | Incomplete sections.                    | All required sections (overview, design, implementation) are complete. | Comprehensive, insightful, and well-written report with proper citations. |       |
| **Contribution & Teamwork**   | No clear individual contribution (if team). | Limited contribution (if team).         | Clear individual contribution (if team); effective teamwork. | Outstanding contribution; actively supports team; leadership demonstrated. |       |

**Total Score**: /100