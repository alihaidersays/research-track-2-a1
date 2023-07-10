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
                plugin='rt2a1_ros2::MinimalPositionServer',
                name='talker'),
            ComposableNode(
                package='rt2a1_ros2',
                plugin='rt2a1_ros2::MinimalStateMachine',
                name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
