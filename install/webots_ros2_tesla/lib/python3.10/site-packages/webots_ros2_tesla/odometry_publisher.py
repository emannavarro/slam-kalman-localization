import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simulation values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        self.create_timer(0.1, self.publish_odometry)

    def publish_odometry(self):
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # Simulated values (replace these with actual robot velocities if available)
        linear_velocity = 1.0  # m/s
        angular_velocity = 0.1  # rad/s

        # Compute new position
        self.x += linear_velocity * delta_time * math.cos(self.theta)
        self.y += linear_velocity * delta_time * math.sin(self.theta)
        self.theta += angular_velocity * delta_time

        # Create quaternion from yaw
        odom_quat = Quaternion()
        odom_quat.z = math.sin(self.theta / 2.0)
        odom_quat.w = math.cos(self.theta / 2.0)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = odom_quat

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom_quat

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
