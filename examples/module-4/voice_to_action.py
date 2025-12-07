# Example: End-to-End Voice Command to Robot Action

This example conceptually demonstrates the full Vision-Language-Action (VLA) pipeline, from a simulated voice command to a simulated robot action. It integrates the concepts from the `WhisperNode` (voice-to-text), `LLMCommandParser` (LLM planning), and `PerceptionMotionIntegrator` (motion planning from perception) into a single, simplified flow.

## Goal

To illustrate the sequential and interconnected nature of a VLA system where a natural language instruction leads to a physical robot behavior in a simulated environment.

## Prerequisites

*   Conceptual understanding of all previous sections in Module 4.
*   No live hardware or complex simulation required for this conceptual example.

## Example File

### `voice_to_action.py` (Python ROS 2 Node - Conceptual Integration)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time

class EndToEndVLANode(Node):
    def __init__(self):
        super().__init__('end_to_end_vla_node')
        self.get_logger().info('End-to-End VLA Node initialized.')

        # Publishers
        self.audio_input_publisher = self.create_publisher(String, '/audio/raw', 10) # Simulated audio input
        self.llm_output_publisher = self.create_publisher(String, '/robot/llm_output', 10) # Simulated LLM output
        self.object_pose_publisher = self.create_publisher(PoseStamped, '/robot/object_poses', 10) # Simulated object pose

        # Subscribers (from conceptual nodes)
        self.create_subscription(String, '/robot/voice_command', self.handle_voice_command, 10)
        self.create_subscription(String, '/robot/actions', self.handle_llm_parsed_action, 10)
        self.create_subscription(String, '/motion_planner/goal_pose', self.handle_motion_plan_request, 10)

        # --- Simulation Logic ---
        # Simulate voice command
        self.voice_command_timer = self.create_timer(7.0, self.simulate_voice_command)
        self.simulated_voice_commands = [
            "Please pick up the blue cube from the table and place it into the red basket.",
            "Move the robot arm to the home position.",
            "What do you see on the left side?",
            "Can you fetch the tool next to the robot?"
        ]
        self.cmd_idx = 0

        # Simulate object pose
        self.object_pose_timer = self.create_timer(3.0, self.simulate_object_pose)
        self.simulated_object_poses = [
            PoseStamped(), # Blue cube
            PoseStamped(), # Red basket
            PoseStamped()  # Tool
        ]
        
        # Initialize mock data
        self.simulated_object_poses[0].header.frame_id = "blue_cube"
        self.simulated_object_poses[0].pose.position.x = 0.6; self.simulated_object_poses[0].pose.position.y = 0.1; self.simulated_object_poses[0].pose.position.z = 0.05
        self.simulated_object_poses[0].pose.orientation.w = 1.0

        self.simulated_object_poses[1].header.frame_id = "red_basket"
        self.simulated_object_poses[1].pose.position.x = 0.8; self.simulated_object_poses[1].pose.position.y = -0.3; self.simulated_object_poses[1].pose.position.z = 0.1
        self.simulated_object_poses[1].pose.orientation.w = 1.0

        self.simulated_object_poses[2].header.frame_id = "tool"
        self.simulated_object_poses[2].pose.position.x = 0.4; self.simulated_object_poses[2].pose.position.y = 0.3; self.simulated_object_poses[2].pose.position.z = 0.1
        self.simulated_object_poses[2].pose.orientation.w = 1.0
        
        self.pose_idx = 0


    def simulate_voice_command(self):
        cmd = self.simulated_voice_commands[self.cmd_idx % len(self.simulated_voice_commands)]
        audio_msg = String(data=cmd) # Simulate audio data as string for simplicity
        self.audio_input_publisher.publish(audio_msg)
        self.get_logger().info(f"[SIM] Published simulated audio input: '{cmd}'")
        self.cmd_idx += 1

    def simulate_object_pose(self):
        pose_msg = self.simulated_object_poses[self.pose_idx % len(self.simulated_object_poses)]
        # Make a deep copy if modifying (not strictly necessary here)
        self.object_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"[SIM] Published simulated object pose for: '{pose_msg.header.frame_id}'")
        self.pose_idx += 1

    # --- Handlers for pipeline stages ---
    def handle_voice_command(self, msg: String):
        self.get_logger().info(f"[Whisper] Received voice command: '{msg.data}' -> Sending to LLM...")
        # Simulate LLM processing and output
        # In a real system, an LLM would process this, potentially with object context
        if "pick up the blue cube" in msg.data.lower():
            llm_action = json.dumps([
                {"action": "say", "phrase": "Attempting to pick up the blue cube."},
                {"action": "move_to_object", "object_name": "blue_cube"},
                {"action": "open_gripper"},
                {"action": "grasp_object", "object_name": "blue_cube"},
                {"action": "move_to_object", "object_name": "red_basket"},
                {"action": "place_object", "location": "red_basket"},
                {"action": "close_gripper"},
                {"action": "say", "phrase": "Blue cube placed in red basket."}
            ])
        elif "move the robot arm to the home position" in msg.data.lower():
            llm_action = json.dumps([
                {"action": "say", "phrase": "Moving arm to home position."},
                {"action": "move_arm_to_pose", "pose_name": "home"}
            ])
        elif "what do you see on the left side" in msg.data.lower():
             llm_action = json.dumps([
                {"action": "say", "phrase": "I see a blue cube and a red basket on the left side."},
                {"action": "describe_scene", "area": "left"}
            ])
        else:
            llm_action = json.dumps([
                {"action": "say", "phrase": "I did not understand that command. Please rephrase."}
            ])
        
        self.llm_output_publisher.publish(String(data=llm_action))
        self.get_logger().info(f"[LLM] Simulated LLM output for: '{msg.data}'")


    def handle_llm_parsed_action(self, msg: String):
        action_dict = json.loads(msg.data)
        action_type = action_dict.get("action")
        self.get_logger().info(f"[Parser] LLM parsed action received: {action_dict}")

        if action_type == "move_to_object":
            object_name = action_dict.get("object_name")
            self.get_logger().info(f"[Planner] Requesting motion to {object_name}...")
            # Simulate publishing a PoseStamped goal from self.simulated_object_poses
            if object_name == "blue_cube":
                self.handle_motion_plan_request(String(data=f"MoveIt! to {object_name} pose")) # Simplified
            elif object_name == "red_basket":
                self.handle_motion_plan_request(String(data=f"MoveIt! to {object_name} pose")) # Simplified
        elif action_type == "grasp_object":
            self.get_logger().info(f"[Robot] Simulating grasping {action_dict.get('object_name')}")
        elif action_type == "place_object":
            self.get_logger().info(f"[Robot] Simulating placing object at {action_dict.get('location')}")
        elif action_type == "say":
            self.get_logger().info(f"[TTS] Robot says: {action_dict.get('phrase')}")
        elif action_type == "move_arm_to_pose":
            self.get_logger().info(f"[Robot] Simulating moving arm to {action_dict.get('pose_name')} pose.")
        elif action_type == "describe_scene":
            self.get_logger().info(f"[Robot] Simulating describing scene for area: {action_dict.get('area')}. Objects visible: blue cube, red basket.")

    def handle_motion_plan_request(self, msg: String):
        # This would usually be a MoveIt plan and execute
        self.get_logger().info(f"[Robot Controller] Executing simulated motion: {msg.data}")
        # Here, actual robot commands would be sent to joint controllers, etc.
        time.sleep(1.0) # Simulate motion time


def main(args=None):
    rclpy.init(args=args)
    end_to_end_node = EndToEndVLANode()
    rclpy.spin(end_to_end_node)
    end_to_end_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
