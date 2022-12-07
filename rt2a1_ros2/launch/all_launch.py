import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rt2a1_ros2',
                plugin='rt2a1_ros2::MiniPublisher',
                name='talker'),
            ComposableNode(
                package='rt2a1_ros2',
                plugin='rt2a1_ros2::MiniSubscriber',
                name='listener'),
            ComposableNode(
                package='rt2a1_ros2',
                plugin='rt2a1_ros2::MiniServer',
                name='server'),
            ComposableNode(
                package='rt2a1_ros2',
                plugin='rt2a1_ros2::MiniClient',
                name='client')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])

