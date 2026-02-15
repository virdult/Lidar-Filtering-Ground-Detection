import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Path to the YAML config file
    config_file = os.path.join(
        get_package_share_directory('lidar_filtering'),
        'config',
        'lidar_filtering.yaml'
    )

    # Load the component into a container
    container = ComposableNodeContainer(
        name='lidar_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_filtering',
                plugin='lidar_filtering::SlopeGroundFilter',
                name='slope_ground_filter',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])