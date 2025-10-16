#!/usr/bin/env python3
"""
science_node.py
When mission_mode == 'science' this node moves the turtle to pre-defined waypoints.
On reaching a waypoint it saves a camera image and logs the latest GNSS fix to disk.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import os
import time
from typing import Tuple, Optional

class ScienceNode(Node):
    def __init__(self) -> None:
        super().__init__('science_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.mode_sub = self.create_subscription(String, 'mission_mode', self.mode_cb, qos)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_cb, qos)
        self.img_sub = self.create_subscription(Image, 'camera/image_raw', self.img_cb, qos)
        self.gnss_sub = self.create_subscription(String, 'gnss/fix', self.gnss_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)
        self.bridge = CvBridge()

        # Waypoints (turtlesim coordinates)
        self.waypoints = [(5.0, 5.0), (8.0, 8.0)]
        self.current_wp = 0
        self.active = False
        self.reached = False
        self.last_gnss: Optional[str] = None
        self.storage_dir = os.path.join(os.path.expanduser('~'), 'rover_science_data')
        os.makedirs(self.storage_dir, exist_ok=True)
        self.get_logger().info(f'Science node will save to {self.storage_dir}')

    def mode_cb(self, msg: String) -> None:
        self.active = (msg.data == 'science')
        self.get_logger().info(f'Science active: {self.active}')

    def gnss_cb(self, msg: String) -> None:
        # store latest GNSS for logging when we reach a site
        self.last_gnss = msg.data

    def pose_cb(self, msg: Pose) -> None:
        if not self.active:
            return
        tx, ty = msg.x, msg.y
        wx, wy = self.waypoints[self.current_wp]
        dx, dy = wx - tx, wy - ty
        dist2 = dx * dx + dy * dy
        # simple proportional steer (turtlesim accepts linear.x and angular.z; we will use simple approach)
        if dist2 < 0.09:  # ~0.3m radius
            if not self.reached:
                self.get_logger().info(f'Reached science site {self.current_wp}: turtle=({tx:.2f},{ty:.2f})')
                self.reached = True
            # stop turtle
            stop = Twist()
            self.cmd_pub.publish(stop)
        else:
            # move towards waypoint (simple)
            t = Twist()
            # use dx, dy to build a naive linear.x & angular.z (turtlesim expects angular.z rotation)
            # compute heading:
            import math
            desired_theta = math.atan2(dy, dx)
            # set a proportional linear velocity based on distance
            linear = min(2.0, 0.8 * (dist2 ** 0.5))
            t.linear.x = linear
            # set angular based on difference to face target
            angular = desired_theta - msg.theta
            # normalize angle
            while angular > math.pi:
                angular -= 2 * math.pi
            while angular < -math.pi:
                angular += 2 * math.pi
            t.angular.z = 2.0 * angular
            self.cmd_pub.publish(t)

    def img_cb(self, img_msg: Image) -> None:
        # Save an image + GNSS when we've flagged 'reached'
        if not (self.active and self.reached):
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            timestamp = int(time.time())
            img_path = os.path.join(self.storage_dir, f'science_site_{self.current_wp}_{timestamp}.png')
            import cv2
            cv2.imwrite(img_path, cv_img)
            # save GNSS text
            gnss_path = os.path.join(self.storage_dir, f'science_site_{self.current_wp}_{timestamp}.txt')
            with open(gnss_path, 'w') as f:
                f.write(self.last_gnss or 'GNSS_UNKNOWN')
            self.get_logger().info(f'Saved image {img_path} and GNSS {gnss_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')
        finally:
            # advance to next waypoint
            self.reached = False
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScienceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
