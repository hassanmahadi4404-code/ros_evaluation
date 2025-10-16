#!/usr/bin/env python3
"""
autonomous_nav_node.py
When mission_mode == 'autonomous' the node autonomously drives the turtle through a sequence
of waypoints and publishes LED status. It also listens to camera images for simple color-based
vision targets.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from typing import List, Tuple
import math
import numpy as np

class AutonomousNav(Node):
    def __init__(self) -> None:
        super().__init__('autonomous_nav_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.mode_sub = self.create_subscription(String, 'mission_mode', self.mode_cb, qos)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_cb, qos)
        self.img_sub = self.create_subscription(Image, 'camera/image_raw', self.img_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)
        self.led_pub = self.create_publisher(String, 'led/status', qos)
        self.bridge = CvBridge()

        # waypoints: two GNSS (simulated) and two vision (we will just use positions)
        self.waypoints: List[Tuple[float, float]] = [(2.0, 2.0), (9.0, 2.0), (9.0, 9.0), (2.0, 9.0)]
        self.current = 0
        self.active = False
        self.get_logger().info('AutonomousNav initialized')

    def mode_cb(self, msg: String) -> None:
        was_active = self.active
        self.active = (msg.data == 'autonomous')
        if self.active and not was_active:
            # just entered autonomous mode
            self.led_pub.publish(String(data='autonomous'))
            self.get_logger().info('Entering autonomous mode')
        if not self.active and was_active:
            self.led_pub.publish(String(data='manual'))
            self.get_logger().info('Leaving autonomous mode')

    def pose_cb(self, msg: Pose) -> None:
        if not self.active:
            return
        tx, ty, theta = msg.x, msg.y, msg.theta
        wx, wy = self.waypoints[self.current]
        dx, dy = wx - tx, wy - ty
        dist2 = dx * dx + dy * dy
        if dist2 < 0.25:
            self.get_logger().info(f'Reached waypoint {self.current} ({wx:.2f},{wy:.2f})')
            self.led_pub.publish(String(data='target_reached'))
            self.current = (self.current + 1) % len(self.waypoints)
            return
        # compute simple proportional controller
        desired_theta = math.atan2(dy, dx)
        angle_diff = desired_theta - theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        t = Twist()
        t.linear.x = min(2.0, 1.0 * math.sqrt(dist2))
        t.angular.z = 2.0 * angle_diff
        self.cmd_pub.publish(t)

    def img_cb(self, img_msg: Image) -> None:
        # simple color check for green vision target; if detected, log and optionally add a waypoint
        if not self.active:
            return
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # sample central rectangle
        crop = cv_img[80:160, 120:200]
        mean_color = crop.mean(axis=(0, 1))  # BGR
        b, g, r = mean_color
        if (g > 100) and (g > r) and (g > b):
            self.get_logger().info('Detected green vision target (simulated)')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
