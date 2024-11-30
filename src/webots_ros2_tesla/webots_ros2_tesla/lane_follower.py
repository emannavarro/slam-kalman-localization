import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node

CONTROL_COEFFICIENT = 0.0005
MIN_DISTANCE_THRESHOLD = 5.0  # Adjust based on your environment

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # ROS interface
        self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)

        # Camera subscription
        qos_camera_data = qos_profile_sensor_data
        qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(Image, 'vehicle/camera/image_color', self.__on_camera_image, qos_camera_data)

        # Lidar subscription
        qos_lidar = qos_profile_sensor_data
        qos_lidar.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(LaserScan, '/webots/lidar', self.__on_lidar_data, qos_lidar)

        # Initialize variables
        self.lidar_data = None
        self.steering_angle = 0.0
        self.speed = 50.0

    def __on_lidar_data(self, message):
        self.lidar_data = message

    def __on_camera_image(self, message):
        img = message.data
        img = np.frombuffer(img, dtype=np.uint8).reshape((message.height, message.width, 4))
        img = img[160:190, :]

        # Segment the image by color in HSV color space
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img, np.array([50, 110, 150]), np.array([120, 255, 255]))

        # Find the largest segmented contour
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_center = cv2.moments(largest_contour)

            if largest_contour_center['m00'] != 0:
                center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
                # Find error (the lane distance from the target distance)
                error = center_x - 190
                self.steering_angle = error * CONTROL_COEFFICIENT
        else:
            # If no lane is detected, keep the steering angle unchanged
            pass

        # Adjust speed and steering based on Lidar data
        self.adjust_control_based_on_lidar()

        # Publish the command
        command_message = AckermannDrive()
        command_message.speed = self.speed
        command_message.steering_angle = self.steering_angle
        self.__ackermann_publisher.publish(command_message)

    def adjust_control_based_on_lidar(self):
        if self.lidar_data is None:
            # If no Lidar data received yet, do nothing
            return

        # Process Lidar data to detect obstacles ahead
        ranges = np.array(self.lidar_data.ranges)
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment
        num_readings = len(ranges)
        angles = angle_min + np.arange(num_readings) * angle_increment

        # Mask invalid readings (e.g., inf or NaN)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Define sectors (e.g., left, front-left, front, front-right, right)
        sectors = {
            'left': {'angles': [], 'distances': []},
            'front_left': {'angles': [], 'distances': []},
            'front': {'angles': [], 'distances': []},
            'front_right': {'angles': [], 'distances': []},
            'right': {'angles': [], 'distances': []}
        }

        # Segment the Lidar data into sectors
        for angle, distance in zip(angles, ranges):
            if np.radians(60) < angle <= np.radians(90):
                sectors['left']['angles'].append(angle)
                sectors['left']['distances'].append(distance)
            elif np.radians(20) < angle <= np.radians(60):
                sectors['front_left']['angles'].append(angle)
                sectors['front_left']['distances'].append(distance)
            elif -np.radians(20) <= angle <= np.radians(20):
                sectors['front']['angles'].append(angle)
                sectors['front']['distances'].append(distance)
            elif -np.radians(60) <= angle < -np.radians(20):
                sectors['front_right']['angles'].append(angle)
                sectors['front_right']['distances'].append(distance)
            elif -np.radians(90) <= angle < -np.radians(60):
                sectors['right']['angles'].append(angle)
                sectors['right']['distances'].append(distance)

        # For each sector, compute the average distance
        sector_distances = {}
        for sector_name, data in sectors.items():
            if data['distances']:
                sector_distances[sector_name] = np.mean(data['distances'])
            else:
                sector_distances[sector_name] = np.inf  # No data, assume no obstacles

        # Decide on action based on sector distances
        # Prioritize sectors with longer distances
        # If obstacle in front, steer towards the side with more space
        min_front_dist = sector_distances['front']

        if min_front_dist < MIN_DISTANCE_THRESHOLD:
            # Obstacle detected ahead
            self.speed = 20.0  # Slow down

            # Decide direction to steer
            if sector_distances['front_left'] > sector_distances['front_right']:
                # Turn left
                avoidance_steering = 0.5  # Adjust as needed
            else:
                # Turn right
                avoidance_steering = -0.5  # Adjust as needed

            # Combine with existing steering angle (from lane following)
            # Weight the obstacle avoidance more when the obstacle is closer
            obstacle_weight = (MIN_DISTANCE_THRESHOLD - min_front_dist) / MIN_DISTANCE_THRESHOLD
            obstacle_weight = np.clip(obstacle_weight, 0.0, 1.0)
            self.steering_angle = (
                (1 - obstacle_weight) * self.steering_angle +
                obstacle_weight * avoidance_steering
            )
        else:
            # No obstacle ahead
            self.speed = 50.0  # Maintain normal speed
            # No adjustment to steering angle

def main(args=None):
    rclpy.init(args=args)
    follower = LaneFollower()
    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
