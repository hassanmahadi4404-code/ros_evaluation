#!/usr/bin/env python3
"""
camera_node.py
Publishes a dummy image stream on /camera/image_raw. The images contain a colored rectangle
that cycles between colors to simulate vision targets.
Requires: cv_bridge, numpy, opencv-python (for saving/debugging).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import numpy as np
from typing import Tuple

class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_node')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Image, 'camera/image_raw', qos)
        self.br = CvBridge()
        self.counter = 0
        self.timer = self.create_timer(0.5, self.publish_dummy_image)  # 2 Hz

    def _make_image(self, size: Tuple[int, int] = (320, 240)) -> np.ndarray:
        h, w = size[1], size[0]
        img = np.zeros((h, w, 3), dtype=np.uint8)
        # cycle rectangle color: green, red, blue
        cycle = self.counter % 3
        if cycle == 0:
            color = (0, 255, 0)  # green
        elif cycle == 1:
            color = (0, 0, 255)  # red (BGR)
        else:
            color = (255, 0, 0)  # blue
        img[60:180, 100:220] = color
        self.counter += 1
        return img

    def publish_dummy_image(self) -> None:
        img = self._make_image()
        msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)
        self.get_logger().debug('Published dummy image')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
