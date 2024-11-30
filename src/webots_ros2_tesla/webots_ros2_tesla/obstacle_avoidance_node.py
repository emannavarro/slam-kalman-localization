#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscribe to Lidar data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/webots/lidar',  # Update this if your topic name is different
            self.lidar_callback,
            10)

        # Publisher for AckermannDrive commands
        self.cmd_publisher = self.create_publisher(
            AckermannDrive,
            'cmd_ackermann',  # Match the topic the tesla_driver node subscribes to
            10)

        # Initialize variables
        self.scan_data = None

        # Create a timer to control the robot at a fixed rate
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def lidar_callback(self, msg):
        self.scan_data = msg

    def control_loop(self):
        if self.scan_data is None:
            # If no scan data received yet, do nothing
            return

        # Process the scan data to find obstacles
        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        angle_increment = self.scan_data.angle_increment
        num_readings = len(ranges)
        angles = angle_min + np.arange(num_readings) * angle_increment

        # Define regions
        front_indices = np.where(np.abs(angles) < np.pi / 6)[0]  # Front 60 degrees
        left_indices = np.where((angles > np.pi / 6) & (angles < np.pi / 2))[0]
        right_indices = np.where((angles < -np.pi / 6) & (angles > -np.pi / 2))[0]

        front_distances = ranges[front_indices]
        left_distances = ranges[left_indices]
        right_distances = ranges[right_indices]

        # Define minimum distances
        min_front_dist = np.min(front_distances)
        min_left_dist = np.min(left_distances)
        min_right_dist = np.min(right_distances)

        # Initialize AckermannDrive command
        cmd_msg = AckermannDrive()
        cmd_msg.speed = 50.0  # Set a suitable speed for your simulation

        # Simple obstacle avoidance logic
        min_distance_threshold = 5.0  # Adjust based on your environment

        if min_front_dist < min_distance_threshold:
            # Obstacle detected in front, decide to turn left or right
            cmd_msg.speed = 20.0  # Slow down
            if min_left_dist > min_right_dist:
                cmd_msg.steering_angle = 0.5  # Turn left
            else:
                cmd_msg.steering_angle = -0.5  # Turn right
        else:
            # No obstacle in front, go straight
            cmd_msg.steering_angle = 0.0

        # Publish the AckermannDrive command
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
