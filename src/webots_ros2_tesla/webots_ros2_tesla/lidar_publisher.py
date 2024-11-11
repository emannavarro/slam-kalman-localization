import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        
        # Publisher for PointCloud2 data
        self.publisher_ = self.create_publisher(PointCloud2, '/lidar/point_cloud', 5)
        self.get_logger().info("Initialized LidarPublisher node")

        # Subscription to the Webots Lidar data (assuming it's published as LaserScan)
        self.subscription = self.create_subscription(
            LaserScan,
            '/webots/lidar',  # Update if this topic name is different
            self.lidar_callback,
            10
        )
        self.get_logger().info("Subscribed to /webots/lidar")

    def lidar_callback(self, scan):
        self.get_logger().info("Lidar data received")
        # Convert LaserScan data to PointCloud2 format
        point_cloud_msg = self.convert_laser_scan_to_point_cloud(scan)
        if point_cloud_msg is not None:
            self.publisher_.publish(point_cloud_msg)
            self.get_logger().info("Published PointCloud2 message")

    def convert_laser_scan_to_point_cloud(self, scan):
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0
                points.append([x, y, z])
            angle += scan.angle_increment

        if not points:
            self.get_logger().warning("No valid points in scan data")
            return None

        # Create PointCloud2 message
        header = scan.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        return point_cloud_msg

def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
    rclpy.spin(lidar_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
