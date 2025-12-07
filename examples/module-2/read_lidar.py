import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        # Create a subscriber to the /scan topic (commonly used for LiDAR data)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('LiDAR Subscriber Node has been started.')

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for receiving LaserScan messages.
        Processes the LiDAR data and prints relevant information.
        """
        self.get_logger().info(f'Received new LaserScan message. Header: {msg.header.stamp}')
        
        # Determine the angle increment
        angle_increment_deg = math.degrees(msg.angle_increment)
        
        # Find the minimum and maximum range readings
        min_range = float('inf')
        max_range = float('-inf')
        
        # Iterate through the ranges to find min/max and some interesting points
        for i, r in enumerate(msg.ranges):
            if not math.isinf(r) and not math.isnan(r):
                if r < min_range:
                    min_range = r
                if r > max_range:
                    max_range = r
                
                # Optionally, print ranges at specific angles (e.g., front, left, right)
                # angle_rad = msg.angle_min + i * msg.angle_increment
                # angle_deg = math.degrees(angle_rad)
                # if -5 < angle_deg < 5: # Around front
                #    self.get_logger().info(f'  Front-ish range ({angle_deg:.1f} deg): {r:.2f} m')
                # if 85 < angle_deg < 95: # Around left
                #    self.get_logger().info(f'  Left-ish range ({angle_deg:.1f} deg): {r:.2f} m')
                # if -95 < angle_deg < -85: # Around right
                #    self.get_logger().info(f'  Right-ish range ({angle_deg:.1f} deg): {r:.2f} m')
        
        if math.isinf(min_range):
            self.get_logger().info('  No valid range readings found.')
        else:
            self.get_logger().info(f'  Min Range: {min_range:.2f} m')
            self.get_logger().info(f'  Max Range: {max_range:.2f} m')
        
        # Example: Get range directly in front (assuming 0 degrees is front)
        # This assumes the scan_callback provides ranges centered around 0 degrees
        try:
            center_index = len(msg.ranges) // 2
            center_range = msg.ranges[center_index]
            if not math.isinf(center_range) and not math.isnan(center_range):
                self.get_logger().info(f'  Range directly ahead: {center_range:.2f} m')
            else:
                self.get_logger().info('  Range directly ahead: Obstacle too close or out of range.')
        except IndexError:
            self.get_logger().warn('  Could not determine center range.')


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
