#!/usr/bin/env python3
"""
mission_manager.py
Publishes the current mission mode and exposes a service to change it at runtime
(no node restart required). Other nodes subscribe to 'mission_mode' to enable/disable behaviour.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import List

MISSION_ORDER: List[str] = ['idle', 'science', 'delivery', 'autonomous']


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.mode_pub = self.create_publisher(String, 'mission_mode', qos)
        self.declare_parameter('initial_mode', 'idle')
        self.current_mode = self.get_parameter('initial_mode').get_parameter_value().string_value

        # Service to cycle mission (simple Trigger). In practice you can create a custom srv
        self.srv = self.create_service(Trigger, 'set_mission', self.set_mission_callback)

        # Publish initial mode once
        self.publish_mode()
        self.get_logger().info(f'Initialized mission_manager, mode={self.current_mode}')

    def publish_mode(self) -> None:
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def set_mission_callback(self, request, response) -> Trigger.Response:
        # Cycle through missions for convenience
        try:
            idx = MISSION_ORDER.index(self.current_mode)
        except ValueError:
            idx = 0
        idx = (idx + 1) % len(MISSION_ORDER)
        self.current_mode = MISSION_ORDER[idx]
        self.publish_mode()
        response.success = True
        response.message = f'Changed mission to {self.current_mode}'
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
