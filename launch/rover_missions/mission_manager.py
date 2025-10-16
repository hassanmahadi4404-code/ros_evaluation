import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionManager(Node):
def __init__(self):
super().__init__('mission_manager')
self.mode_pub = self.create_publisher(String, 'mission_mode', 10)
self.declare_parameter('initial_mode', 'idle')
self.current_mode = self.get_parameter('initial_mode').get_parameter_value().string_value


# Service to change mission
self.srv = self.create_service(Trigger, 'set_mission', self.set_mission_callback)
self.get_logger().info(f'Initial mission mode: {self.current_mode}')


# Publish initial mode
msg = String()
msg.data = self.current_mode
self.mode_pub.publish(msg)


def set_mission_callback(self, request, response):
# For simplicity, use request.success boolean to toggle through modes (alternative: custom srv)
# We'll cycle: idle -> science -> delivery -> autonomous -> idle
order = ['idle', 'science', 'delivery', 'autonomous']
idx = order.index(self.current_mode) if self.current_mode in order else 0
idx = (idx + 1) % len(order)
self.current_mode = order[idx]
msg = String(); msg.data = self.current_mode
self.mode_pub.publish(msg)
response.success = True
response.message = f'Changed mission to {self.current_mode}'
self.get_logger().info(response.message)
return response


def main(args=None):
rclpy.init(args=args)
node = MissionManager()
try:
rclpy.spin(node)
except KeyboardInterrupt:
pass
node.destroy_node()
rclpy.shutdown()


if __name__=='__main__':
main()
