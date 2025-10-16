#!/usr/bin/env python3
"""
equipment_servicing_node.py
Simulates the Equipment Servicing Mission:
- Delivering a cached science sample to a lander.
- Performing maintenance (press button, toggle switch, etc.).
- Uses simple motion control and publishes mission logs.
Compatible with ROS 2 Jazzy.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time


class EquipmentServicingNode(Node):
    def __init__(self):
        super().__init__('equipment_servicing_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # Subscriptions
        self.mode_sub = self.create_subscription(String, 'mission_mode', self.mode_callback, qos)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, qos)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', qos)
        self.delivery_pub = self.create_publisher(String, 'lander/delivery_status', qos)
        self.maint_pub = self.create_publisher(String, 'lander/maintenance_status', qos)

        # Mission state
        self.active = False
        self.stage = 'idle'
        self.lander_pos = (6.0, 6.0)  # target position (lander)
        self.delivered = False
        self.maint_done = False

        self.get_logger().info('Equipment Servicing Node initialized.')

    # ---------- Callbacks ---------- #
    def mode_callback(self, msg: String):
        """Activate/deactivate based on mission mode."""
        if msg.data == 'equipment_servicing':
            if not self.active:
                self.active = True
                self.stage = 'navigate'
                self.get_logger().info('Equipment Servicing mission activated.')
        else:
            self.active = False
            self.stage = 'idle'

    def pose_callback(self, msg: Pose):
        """Main mission logic."""
        if not self.active:
            return

        if self.stage == 'navigate':
            self.navigate_to_lander(msg)
        elif self.stage == 'deliver':
            self.deliver_sample()
        elif self.stage == 'maintain':
            self.perform_maintenance()
        elif self.stage == 'done':
            self.finish_mission()

    # ---------- Mission Behaviors ---------- #
    def navigate_to_lander(self, pose: Pose):
        """Drive turtle to the lander."""
        tx, ty, theta = pose.x, pose.y, pose.theta
        lx, ly = self.lander_pos
        dx, dy = lx - tx, ly - ty
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.3:
            self.get_logger().info('Reached lander site. Starting delivery.')
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.stage = 'deliver'
            return

        # Move toward lander
        desired_theta = math.atan2(dy, dx)
        angle_diff = desired_theta - theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd = Twist()
        cmd.linear.x = min(2.0, 1.0 * dist)
        cmd.angular.z = 2.0 * angle_diff
        self.cmd_pub.publish(cmd)

    def deliver_sample(self):
        """Simulate delivering a cached science sample."""
        if not self.delivered:
            msg = String()
            msg.data = 'Sample delivered successfully.'
            self.delivery_pub.publish(msg)
            self.get_logger().info(msg.data)
            self.delivered = True
            # Move to next stage after delay
            time.sleep(1.0)
            self.stage = 'maintain'

    def perform_maintenance(self):
        """Simulate lander maintenance."""
        if not self.maint_done:
            steps = [
                'Extending robotic arm...',
                'Pressing diagnostic button...',
                'Tightening valve...',
                'Checking status lights...'
            ]
            for s in steps:
                self.get_logger().info(s)
                msg = String()
                msg.data = s
                self.maint_pub.publish(msg)
                time.sleep(1.0)

            done_msg = String()
            done_msg.data = 'Maintenance completed.'
            self.maint_pub.publish(done_msg)
            self.get_logger().info(done_msg.data)
            self.maint_done = True
            self.stage = 'done'

    def finish_mission(self):
        """Stop and report mission completion."""
        stop = Twist()
        self.cmd_pub.publish(stop)
        final_msg = String()
        final_msg.data = 'Equipment Servicing mission complete!'
        self.maint_pub.publish(final_msg)
        self.get_logger().info(final_msg.data)
        self.stage = 'idle'
        self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = EquipmentServicingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
