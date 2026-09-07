"""Fixed-map localization; external robot or bag supplies odometry and sensor TF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    config = value('params_file')
    if not os.path.isfile(config):
        raise RuntimeError(f'params_file must be a YAML file, not a directory: {config}')
    sim_time = value('use_sim_time').lower() == 'true'
    overrides = {'use_sim_time': sim_time}
    for name in ('map_path', 'cloud_topic', 'odom_topic', 'map_frame', 'odom_frame', 'base_frame'):
        if value(name):
            overrides[name] = value(name)
    nodes = [Node(
        package='wheel_lidar_localization', executable='localization_node',
        name='wheel_lidar_localization', output='screen', parameters=[config, overrides])]
    if value('rviz').lower() == 'true':
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='localization_rviz',
            arguments=['-d', value('rviz_config')], parameters=[{'use_sim_time': sim_time}],
            output='screen'))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('wheel_lidar_localization')
    args = [
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'rfans_wheel.yaml'),
                              description='Full YAML file path'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'localization.rviz')),
    ]
    for name in ('map_path', 'cloud_topic', 'odom_topic', 'map_frame', 'odom_frame', 'base_frame'):
        args.append(DeclareLaunchArgument(name, default_value='', description='Optional YAML override'))
    return LaunchDescription(args + [OpaqueFunction(function=create_nodes)])
