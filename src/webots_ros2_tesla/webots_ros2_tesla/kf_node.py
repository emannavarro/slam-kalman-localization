#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
import numpy as np
import math
import cv2  # OpenCV for image processing
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kf_node')

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.create_subscription(PointCloud2, '/lidar/points/point_cloud', self.pointcloud_callback, qos_profile)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)

        # Publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/odom_filtered', qos_profile)
        self.landmark_pub = self.create_publisher(MarkerArray, '/landmarks', qos_profile)

        # Initialize state and covariance
        self.state = np.zeros((3, 1))  # [x, y, theta]
        self.P = np.eye(3)

        # Process and measurement noise covariances
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2  # Process noise covariance
        self.R = np.diag([0.5, np.deg2rad(5.0)]) ** 2      # Measurement noise covariance

        # Initialize timestamp
        self.last_time = self.get_clock().now()

        # Landmarks extracted from the map
        self.landmarks = []

        # Parameters for landmark extraction and matching
        self.lidar_range_max = 10.0  # Maximum lidar range
        self.lidar_fov = np.deg2rad(360)  # Lidar field of view
        self.landmark_detection_threshold = 0.5  # Threshold for matching landmarks [m]

        self.get_logger().info("Kalman Filter Node Initialized")

    def map_callback(self, map_msg):
        try:
            self.get_logger().info("Map data received")
            # Convert the occupancy grid data to a numpy array
            width = map_msg.info.width
            height = map_msg.info.height
            data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

            # Process the data to extract landmarks
            self.extract_landmarks_from_map(data, map_msg.info)

        except Exception as e:
            self.get_logger().error(f"Error in map_callback: {e}")

    def extract_landmarks_from_map(self, data, info):
        try:
            # Occupancy data: -1 (unknown), 0 (free), 100 (occupied)
            # We create a binary image where occupied cells are white
            binary_map = np.uint8((data == 100) * 255)

            # Use OpenCV to detect corners (landmarks)
            corners = cv2.goodFeaturesToTrack(
                binary_map,
                maxCorners=100,
                qualityLevel=0.01,
                minDistance=10
            )

            if corners is not None:
                landmarks = []
                for corner in corners:
                    y_pixel, x_pixel = corner.ravel()  # Note: OpenCV uses (x, y)

                    # Convert pixel coordinates to map coordinates
                    x = info.origin.position.x + x_pixel * info.resolution
                    y = info.origin.position.y + y_pixel * info.resolution

                    landmarks.append([x, y])

                self.landmarks = landmarks
                self.get_logger().info(f"Extracted {len(landmarks)} landmarks from the map.")
            else:
                self.landmarks = []
                self.get_logger().warning("No landmarks found in the map.")

            # Publish landmarks for visualization
            self.publish_landmarks()

        except Exception as e:
            self.get_logger().error(f"Error in extract_landmarks_from_map: {e}")

    def publish_landmarks(self):
        marker_array = MarkerArray()
        for i, lm in enumerate(self.landmarks):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "landmarks"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = lm[0]
            marker.pose.position.y = lm[1]
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)

        self.landmark_pub.publish(marker_array)

    def odom_callback(self, msg):
        try:
            # Extract velocities
            v = msg.twist.twist.linear.x
            omega = msg.twist.twist.angular.z

            # Time difference
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # State prediction
            theta = self.state[2, 0]
            self.state[0, 0] += v * dt * math.cos(theta)
            self.state[1, 0] += v * dt * math.sin(theta)
            self.state[2, 0] += omega * dt
            self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi

            # Compute Jacobian of the motion model
            F = np.eye(3)
            F[0, 2] = -v * dt * math.sin(theta)
            F[1, 2] = v * dt * math.cos(theta)

            # Process noise covariance
            Q = self.Q

            # Update covariance
            self.P = F @ self.P @ F.T + Q

        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")

    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 message to numpy array
            lidar_points = self.convert_pointcloud2_to_numpy(msg)

            # Transform lidar points to the map frame using the current estimated pose
            theta = self.state[2, 0]
            rotation_matrix = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]
            ])
            translation = self.state[0:2, 0].reshape(2, 1)
            lidar_points_map_frame = (rotation_matrix @ lidar_points.T).T + translation.T

            # Perform data association between lidar points and landmarks
            observed_landmarks = self.match_landmarks(lidar_points_map_frame)

            # Perform EKF update with these observations
            self.update_step_landmarks(observed_landmarks)

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud_callback: {e}")

    def convert_pointcloud2_to_numpy(self, msg):
        # Extract x, y coordinates from PointCloud2
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            x, y = point[:2]
            points.append([x, y])
        if not points:
            self.get_logger().warning("No valid points in PointCloud2 data.")
        return np.array(points)

    def match_landmarks(self, lidar_points_map_frame):
        observed_landmarks = []

        if not self.landmarks:
            self.get_logger().warning("No landmarks available for matching.")
            return observed_landmarks

        for lm in self.landmarks:
            lm_pos = np.array(lm)
            # Compute distances from lidar points to the landmark
            distances = np.linalg.norm(lidar_points_map_frame - lm_pos, axis=1)
            min_distance = np.min(distances)

            if min_distance < self.landmark_detection_threshold:
                # Landmark detected
                dx = lm_pos[0] - self.state[0, 0]
                dy = lm_pos[1] - self.state[1, 0]
                range_meas = np.sqrt(dx**2 + dy**2)
                bearing_meas = np.arctan2(dy, dx) - self.state[2, 0]
                bearing_meas = (bearing_meas + np.pi) % (2 * np.pi) - np.pi

                observed_landmarks.append({
                    'position': lm_pos,
                    'range': range_meas,
                    'bearing': bearing_meas
                })

        self.get_logger().info(f"Matched {len(observed_landmarks)} landmarks.")
        return observed_landmarks

    def update_step_landmarks(self, observed_landmarks):
        try:
            for obs in observed_landmarks:
                lm_pos = obs['position']
                z_meas = np.array([[obs['range']],
                                   [obs['bearing']]])

                # Expected measurement based on current state
                dx = lm_pos[0] - self.state[0, 0]
                dy = lm_pos[1] - self.state[1, 0]
                q = dx**2 + dy**2
                sqrt_q = np.sqrt(q)
                z_pred = np.array([[sqrt_q],
                                   [np.arctan2(dy, dx) - self.state[2, 0]]])
                z_pred[1, 0] = (z_pred[1, 0] + np.pi) % (2 * np.pi) - np.pi

                # Compute Jacobian H
                H = np.array([
                    [-dx / sqrt_q, -dy / sqrt_q, 0],
                    [dy / q, -dx / q, -1]
                ])

                # Measurement noise covariance
                R = self.R

                # Innovation
                y = z_meas - z_pred
                y[1, 0] = (y[1, 0] + np.pi) % (2 * np.pi) - np.pi  # Normalize bearing

                # Innovation covariance
                S = H @ self.P @ H.T + R

                # Kalman gain
                K = self.P @ H.T @ np.linalg.inv(S)

                # Update state and covariance
                self.state += K @ y
                self.P = (np.eye(3) - K @ H) @ self.P

                # Normalize theta
                self.state[2, 0] = (self.state[2, 0] + np.pi) % (2 * np.pi) - np.pi

            # Publish the updated pose
            self.publish_pose()

        except Exception as e:
            self.get_logger().error(f"Error in update_step_landmarks: {e}")

    def publish_pose(self):
        try:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            pose_msg.pose.pose.position.x = self.state[0, 0]
            pose_msg.pose.pose.position.y = self.state[1, 0]
            pose_msg.pose.pose.position.z = 0.0

            # Convert yaw to quaternion
            theta = self.state[2, 0]
            quaternion = self.yaw_to_quaternion(theta)
            pose_msg.pose.pose.orientation = quaternion

            # Covariance
            P_flat = self.P.flatten()
            pose_msg.pose.covariance = [
                P_flat[0], P_flat[1], 0.0, 0.0, 0.0, P_flat[2],
                P_flat[3], P_flat[4], 0.0, 0.0, 0.0, P_flat[5],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                P_flat[6], P_flat[7], 0.0, 0.0, 0.0, P_flat[8]
            ]

            self.pose_pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_pose: {e}")

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert a yaw angle (in radians) into a quaternion message."""
        half_yaw = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
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
