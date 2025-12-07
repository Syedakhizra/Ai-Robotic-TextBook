# Example: Integrating Perception Output with a Motion Planner

This example conceptually demonstrates how the 6-DoF pose of a detected object (output from a perception pipeline) can be used as a target for a robot's motion planner. In a full Vision-Language-Action (VLA) system, an LLM would interpret a command, the perception system would locate the target, and then the motion planner would execute the physical interaction.

## Goal

To show the data flow from object pose estimation to a motion planning request, focusing on how a robot's end-effector target is derived from the perceived object.

## Prerequisites

*   Conceptual understanding of ROS 2 topics, object detection, and pose estimation.
*   Conceptual understanding of motion planning and inverse kinematics.
*   No live hardware or complex simulation required for this conceptual example.

## Example File

### `perception_motion_integration.py` (Python ROS 2 Node)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json

class PerceptionMotionIntegrator(Node):
    def __init__(self):
        super().__init__('perception_motion_integrator')
        
        # Subscriber to object pose estimation output
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot/object_poses', # Topic publishing detected object poses
            self.object_pose_callback,
            10)
        
        # Publisher for motion planning requests (e.g., to MoveIt 2)
        # This would typically be a specific MoveIt message type (e.g., MoveGroupActionGoal)
        # For simplicity, we'll publish a String representation of the target pose
        self.motion_plan_publisher = self.create_publisher(
            String, # Conceptual: In real app, use appropriate MoveIt msg type
            '/motion_planner/goal_pose',
            10)
        
        self.get_logger().info('Perception Motion Integrator Node initialized.')

        # Simulate object pose for demonstration
        self.timer = self.create_timer(5.0, self.simulate_object_pose)
        self.simulated_poses = [
            PoseStamped(
                header=rclpy.time.Time().to_msg(),
                pose=PoseStamped().pose
            ), # Example empty pose
            PoseStamped(
                header=rclpy.time.Time().to_msg(),
                pose=PoseStamped().pose # Fill with some mock data
            )
        ]
        # Mock data for demonstration:
        self.simulated_poses[0].header.frame_id = "camera_link"
        self.simulated_poses[0].pose.position.x = 0.5
        self.simulated_poses[0].pose.position.y = -0.1
        self.simulated_poses[0].pose.position.z = 0.2
        self.simulated_poses[0].pose.orientation.w = 1.0 # Identity quaternion

        self.simulated_poses[1].header.frame_id = "camera_link"
        self.simulated_poses[1].pose.position.x = 0.3
        self.simulated_poses[1].pose.position.y = 0.2
        self.simulated_poses[1].pose.position.z = 0.1
        self.simulated_poses[1].pose.orientation.w = 0.707
        self.simulated_poses[1].pose.orientation.z = 0.707 # 90 degrees rotation

        self.pose_index = 0


    def simulate_object_pose(self):
        """Simulates receiving an object pose from a perception system."""
        self.object_pose_callback(self.simulated_poses[self.pose_index % len(self.simulated_poses)])
        self.pose_index += 1

    def object_pose_callback(self, msg: PoseStamped):
        """
        Callback function to process detected object poses and formulate a motion planning request.
        """
        self.get_logger().info(f'Received object pose in frame "{msg.header.frame_id}":')
        self.get_logger().info(f'  Position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')
        self.get_logger().info(f'  Orientation: x={msg.pose.orientation.x:.2f}, y={msg.pose.orientation.y:.2f}, z={msg.pose.orientation.z:.2f}, w={msg.pose.orientation.w:.2f}')

        # --- Conceptual Motion Planning Request Logic ---
        # In a real system, this would involve:
        # 1. Transforming the pose to the robot's base frame if necessary.
        # 2. Calculating a pre-grasp, grasp, and post-grasp pose relative to the object.
        # 3. Formulating a MoveIt 2 planning request (e.g., setting an end-effector target pose).
        
        # For this example, we'll simply publish the received pose as a target goal.
        # You might need to adjust the Z-axis for a gripper to approach from above.
        target_pose_for_planner = PoseStamped()
        target_pose_for_planner.header.frame_id = msg.header.frame_id # Or robot_base_link
        target_pose_for_planner.pose.position.x = msg.pose.position.x
        target_pose_for_planner.pose.position.y = msg.pose.position.y
        target_pose_for_planner.pose.position.z = msg.pose.position.z + 0.1 # Move slightly above the object
        target_pose_for_planner.pose.orientation = msg.pose.orientation # Maintain object orientation

        motion_request_msg = String()
        motion_request_msg.data = f"Motion plan request: go to {target_pose_for_planner.pose.position} in {target_pose_for_planner.header.frame_id} frame"
        
        self.motion_plan_publisher.publish(motion_request_msg)
        self.get_logger().info(f'Published conceptual motion plan request: {motion_request_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    integrator_node = PerceptionMotionIntegrator()
    rclpy.spin(integrator_node)
    integrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
