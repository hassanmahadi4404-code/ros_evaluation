#!/usr/bin/env python3
"""
delivery_node.py
Provides an Action Server 'pick_deliver' implementing PickDeliver.action.
Also publishes delivery/status topic when done.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rover_missions.action import PickDeliver  # requires action to be built/installed
import time

class DeliveryNode(Node):
    def __init__(self) -> None:
        super().__init__('delivery_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.mode_sub = self.create_subscription(String, 'mission_mode', self.mode_cb, qos)
        self.status_pub = self.create_publisher(String, 'delivery/status', qos)
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)

        # Action server
        self._action_server = ActionServer(self,
                                          PickDeliver,
                                          'pick_deliver',
                                          execute_callback=self.execute_callback,
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback)
        self.active = False

    def mode_cb(self, msg: String) -> None:
        self.active = (msg.data == 'delivery')
        self.get_logger().info(f'Delivery active: {self.active}')

    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(f'Received PickDeliver goal: pick={goal_request.pick_location} -> deliver={goal_request.deliver_location}')
        # Accept if node is active; otherwise reject
        if not self.active:
            self.get_logger().warn('Rejecting goal because delivery mode is not active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested for goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Synchronous execution for demo; publish feedback
        feedback_msg = PickDeliver.Feedback()
        # 1. Go to pick
        feedback_msg.status = 'Navigating to pick location'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)

        # simulate pick action
        feedback_msg.status = 'Picking object'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)

        # 2. Go to deliver
        feedback_msg.status = 'Delivering object'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)

        # Publish delivery status topic
        result = PickDeliver.Result()
        result.success = True
        result.message = f'Delivered to {goal_handle.request.deliver_location}'
        msg = String()
        msg.data = result.message
        self.status_pub.publish(msg)
        self.get_logger().info(f'Delivery finished: {result.message}')

        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
