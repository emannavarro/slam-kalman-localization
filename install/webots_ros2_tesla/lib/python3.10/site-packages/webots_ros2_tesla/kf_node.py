import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  # Uncommented import
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import numpy as np
import math

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kf_node')

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)  # Uncommented

        # Publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/odom_filtered', 10)

        # Initialize state and covariance
        self.state = np.zeros((3, 1))  # [x, y, theta]
        self.P = np.eye(3)

        # Process and measurement noise covariances
        self.Q = np.diag([0.1, 0.1, np.deg2rad(5.0)])  # Adjust as needed
        self.R = np.diag([0.2, 0.2, np.deg2rad(30.0)])  # Adjusted measurement noise covariance

        # Initialize timestamp
        self.last_time = self.get_clock().now()

        self.create_timer(0.1, self.publish_pose)  # Publish at 10 Hz

    def odom_callback(self, msg):
        try:
            self.get_logger().info("Odometry data received")

            # Extract control inputs (velocities)
            v = msg.twist.twist.linear.x
            omega = msg.twist.twist.angular.z

            # Time update
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # Predict step using a simple motion model
            theta = self.state[2, 0]

            # State transition model (F) and control input model (B)
            F = np.eye(3)
            B = np.array([
                [dt * math.cos(theta), 0],
                [dt * math.sin(theta), 0],
                [0, dt]
            ])
            u = np.array([[v], [omega]])

            # Predict the new state
            self.state = F @ self.state + B @ u
            self.P = F @ self.P @ F.T + self.Q

            # Use odometry data as the measurement
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Convert quaternion to yaw
            orientation_q = msg.pose.pose.orientation
            siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
            cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
            theta_meas = np.arctan2(siny_cosp, cosy_cosp)

            # Create the measurement vector
            z = np.array([[x], [y], [theta_meas]])

            # Perform the measurement update
            self.update_step(z)

            # Publish the updated pose
            self.publish_pose()
        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")



    def scan_callback(self, msg):
        try:
            self.get_logger().info("Scan data received")

            # Extract ranges and angles from the LaserScan message
            ranges = np.array(msg.ranges)
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

            # Filter out invalid ranges (e.g., Inf, NaN, or out of range values)
            valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]

            if len(ranges) < 10:  # Adjust threshold as needed
                self.get_logger().warning("Not enough valid lidar points to compute centroid")
                return

            # Convert polar coordinates to Cartesian coordinates relative to the robot
            x_points = ranges * np.cos(angles)
            y_points = ranges * np.sin(angles)

            # Compute the centroid of the detected points
            x_mean = np.mean(x_points)
            y_mean = np.mean(y_points)

            # Transform centroid to the odom frame
            z_x = self.state[0, 0] + x_mean
            z_y = self.state[1, 0] + y_mean
            z_theta = self.state[2, 0]  # We can't update theta without scan matching

            # Create the measurement vector
            z = np.array([[z_x], [z_y], [z_theta]])

            # Perform the measurement update
            self.update_step(z)
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")



    def update_step(self, z):
        try:
            self.get_logger().info("Update step called")
            # Measurement matrix
            H = np.eye(3)

            # Measurement residual
            y = z - H @ self.state

            # Residual covariance
            S = H @ self.P @ H.T + self.R

            # Kalman gain
            K = self.P @ H.T @ np.linalg.inv(S)

            # Update state estimate
            self.state = self.state + K @ y

            # Update covariance estimate
            self.P = (np.eye(3) - K @ H) @ self.P

            # Publish the updated pose
            self.publish_pose()
        except Exception as e:
            self.get_logger().error(f"Error in update_step: {e}")

    def publish_pose(self):
        try:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "odom"

            pose_msg.pose.pose.position.x = self.state[0, 0]
            pose_msg.pose.pose.position.y = self.state[1, 0]
            pose_msg.pose.pose.position.z = 0.0

            # Convert theta back to quaternion
            theta = self.state[2, 0]
            quaternion = self.yaw_to_quaternion(theta)
            pose_msg.pose.pose.orientation = quaternion

            # Fill in the covariance
            cov = self.P.flatten()
            pose_msg.pose.covariance = [
                cov[0], 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, cov[4], 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, cov[8], 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, cov[0], 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, cov[4], 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, cov[8]
            ]

            self.pose_pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_pose: {e}")

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert a yaw angle (in radians) into a quaternion message."""
        half_yaw = yaw * 0.5
        q = Quaternion()
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

def main(args=None):
    rclpy.init(args=args)
    kf_node = KalmanFilterNode()
    rclpy.spin(kf_node)
    kf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
