#!/usr/bin/env python3
"""
led_sim_node.py
Subscribes to 'led/status' and prints a human-friendly LED color mapping to the terminal.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LEDSim(Node):
    def __init__(self) -> None:
        super().__init__('led_sim_node')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(String, 'led/status', self.cb, qos)

    def cb(self, msg: String) -> None:
        mapping = {
            'autonomous': 'Red',
            'manual': 'Blue',
            'target_reached': 'Green'
        }
        color = mapping.get(msg.data, 'Off')
        self.get_logger().info(f'LED: {color} ({msg.data})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LEDSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
