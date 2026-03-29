"""
Launch file for Kinova Gen3 lifecycle controller.
Loads KinovaLifecycleController as a component into a shared container.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container=ComposableNodeContainer(
        name='kinova_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[ComposableNode(
            package='kinova_controller',
            plugin='kinova_ros2::KinovaLifecycleController',
            name='kinova_controller',
            parameters=[{
                'robot_ip':'192.168.1.10',
                'robot_port':10000
            }]
        )]
    )
    return LaunchDescription([container])