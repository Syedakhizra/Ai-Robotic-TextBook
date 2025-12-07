import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class LLMCommandParser(Node):
    def __init__(self):
        super().__init__('llm_command_parser')
        # Subscriber to the LLM's output (simulated here)
        self.command_subscription = self.create_subscription(
            String,
            '/robot/llm_output', # Topic where LLM output is published
            self.llm_output_callback,
            10)
        
        # Publisher for parsed robot actions
        self.action_publisher = self.create_publisher(String, '/robot/actions', 10)
        
        # Simulate LLM output for demonstration
        self.timer = self.create_timer(5.0, self.simulate_llm_output)
        self.llm_outputs = [
            json.dumps([
                {"action": "say", "phrase": "Okay, I will grab the red ball."},
                {"action": "move_to_object", "object_name": "red ball"},
                {"action": "open_gripper"},
                {"action": "grasp_object", "object_name": "red ball"},
                {"action": "close_gripper"}
            ]),
            json.dumps([
                {"action": "say", "phrase": "Moving to the green box."},
                {"action": "navigate_to_location", "location": "green box"}
            ]),
            json.dumps([
                {"action": "say", "phrase": "I need clarification. Which object do you mean?"},
                {"action": "ask_clarification", "question": "Which object do you mean?"}
            ])
        ]
        self.output_index = 0
        
        self.get_logger().info('LLM Command Parser Node initialized.')

    def simulate_llm_output(self):
        """Simulates receiving LLM output for demonstration purposes."""
        llm_output_str = self.llm_outputs[self.output_index % len(self.llm_outputs)]
        msg = String()
        msg.data = llm_output_str
        self.llm_output_callback(msg) # Directly call the callback with simulated data
        self.output_index += 1

    def llm_output_callback(self, msg: String):
        """
        Callback function to process raw LLM output and parse it into robot actions.
        Expected format: JSON string representing a list of action dictionaries.
        """
        llm_output_str = msg.data
        self.get_logger().info(f'Received LLM output: "{llm_output_str}"')

        try:
            action_list = json.loads(llm_output_str)
            if not isinstance(action_list, list):
                raise ValueError("LLM output is not a list.")

            for action_dict in action_list:
                if not isinstance(action_dict, dict) or "action" not in action_dict:
                    raise ValueError("Each action must be a dictionary with an 'action' key.")
                
                action_type = action_dict["action"]
                # Here, you would implement logic to translate these parsed actions
                # into actual robot commands (e.g., publishing to other topics, calling services)
                
                parsed_action_msg = String()
                parsed_action_msg.data = json.dumps(action_dict)
                self.action_publisher.publish(parsed_action_msg)
                self.get_logger().info(f'Published parsed action: "{action_type}" with params: {json.dumps(action_dict)}')
                time.sleep(0.5) # Simulate some processing time between actions

        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse LLM output as JSON: "{llm_output_str}"')
        except ValueError as e:
            self.get_logger().error(f'Invalid LLM output format: {e} in "{llm_output_str}"')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred during parsing: {e}')


def main(args=None):
    rclpy.init(args=args)
    llm_parser_node = LLMCommandParser()
    rclpy.spin(llm_parser_node)
    llm_parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
