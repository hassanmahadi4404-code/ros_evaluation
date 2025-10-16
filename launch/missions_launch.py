from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
ld = LaunchDescription()


# Start turtlesim
ld.add_action(Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'))


# Mission manager
ld.add_action(Node(package='rover_missions', executable='mission_manager.py', name='mission_manager'))


# Core nodes
ld.add_action(Node(package='rover_missions', executable='camera_node.py', name='camera_node'))
ld.add_action(Node(package='rover_missions', executable='gnss_node.py', name='gnss_node'))
ld.add_action(Node(package='rover_missions', executable='led_sim_node.py', name='led_sim_node'))


ld.add_action(Node(package='rover_missions', executable='science_node.py', name='science_node'))
ld.add_action(Node(package='rover_missions', executable='delivery_node.py', name='delivery_node'))
ld.add_action(Node(package='rover_missions', executable='autonomous_nav_node.py', name='autonomous_nav_node'))


return ld
