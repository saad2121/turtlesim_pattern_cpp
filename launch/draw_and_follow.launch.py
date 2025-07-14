
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtle1'),
        Node(package='turtlesim', executable='turtlesim_node', name='turtle2'),
        #Node(package='turtlesim_pattern_cpp', executable='square_drawer', output='screen'),
        #Node(package='turtlesim_pattern_cpp', executable='follower', output='screen'),
    ])