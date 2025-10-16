#!/usr/bin/env python3
"""
gnss_node.py
Publishes a dummy GNSS fix as a text message on 'gnss/fix'.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class GNSSNode(Node):
    def __init__(self) -> None:
        super().__init__('gnss_node')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, 'gnss/fix', qos)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.publish_fix)  # 1 Hz

    def publish_fix(self) -> None:
        lat = 23.700000 + 0.0005 * self.counter
        lon = 90.400000 + 0.0005 * self.counter
        msg = String()
        msg.data = f'{lat:.6f},{lon:.6f}'
        self.pub.publish(msg)
        self.get_logger().debug(f'GNSS -> {msg.data}')
        self.counter += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GNSSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
