#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
#  yahboom_nav / scripts / publish_initial_pose.py
#
#  One-shot node: reads x, y, yaw from CLI args, publishes a single
#  PoseWithCovarianceStamped to /initialpose, then exits.
#
#  Called automatically by auto_nav.launch.py – no need to run manually.
#  If you want to reset pose mid-mission:
#    ros2 run yahboom_nav publish_initial_pose --ros-args \
#      -p x:=1.0 -p y:=2.0 -p yaw:=1.57
# ─────────────────────────────────────────────────────────────────────────────

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x',   0.0)
        self.declare_parameter('y',   0.0)
        self.declare_parameter('yaw', 0.0)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Small delay so the publisher has time to connect before we send
        self.timer = self.create_timer(0.5, self.publish_and_exit)

    def publish_and_exit(self):
        self.timer.cancel()

        x   = self.get_parameter('x').value
        y   = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Modest covariance – AMCL will refine with the first few scans
        msg.pose.covariance[0]  = 0.25   # xx
        msg.pose.covariance[7]  = 0.25   # yy
        msg.pose.covariance[35] = 0.068  # yaw-yaw (~15°)

        self.pub.publish(msg)
        self.get_logger().info(
            f'[InitialPose] Published → x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°')

        raise SystemExit


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
