"""Autoware Core の自己位置推定の launch と同じ構成で、固定地図上の自己位置推定を起動する。

参照元: autoware_core (commit e0a8ab84)
  localization/autoware_core_localization/launch/autoware_core_localization.launch.xml

  名前空間 / ノード                                     | autoware_core の対応
  ------------------------------------------------------+-------------------------------------------
  /localization/pose_estimator/ndt_scan_matcher          | autoware_ndt_scan_matcher（名前空間も同じ）
  /localization/pose_twist_fusion_filter/ekf_localizer   | autoware_ekf_localizer → robot_localization で代替

  /surestar_points ─▶ ndt_scan_matcher ─▶ /localization/pose_estimator/pose_with_covariance ─┐
                          ▲ /localization/kinematic_state                                      ▼
  /odometry/filtered ─────────────────────────────────────────────────────────▶ ekf_localizer ─▶ TF map->odom
  /initialpose（RViz）─────────────────────────────────────────────────────────────────────────┘

Autoware との違い: voxel_grid_downsample_filter ノードがない（ndt_scan_matcher の中で前処理する）、
twist_estimator / stop_filter / twist2accel がない（外部の odom 用 EKF が /odometry/filtered を出す）、
pose_initializer / initial_pose_adaptor がない（RViz の /initialpose をそのまま EKF の set_pose に渡す。
z/roll/pitch は RViz の値（0）のまま）、map_loader ノードがない（ndt_scan_matcher が PCD を自分で読む）。
odom 用 EKF と base_link → LiDAR の TF は、ロボット本体か bag から与える。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    """launch 引数の値を確定させてから、起動するノードの一覧を作る（OpaqueFunction から呼ばれる）。

    入力: context  launch のコンテキスト。launch 引数の値を取り出すのに使う。
    出力: 起動する Node のリスト（ndt_scan_matcher、ekf_localizer、必要なら rviz2）。
    例外: パラメータファイルのパスが YAML ファイルでない場合は RuntimeError。
    """

    def value(name):
        """launch 引数 name の値を文字列で返す。"""
        return LaunchConfiguration(name).perform(context)

    for name in ('ndt_scan_matcher_param_path', 'ekf_localizer_param_path'):
        if not os.path.isfile(value(name)):
            raise RuntimeError(f'{name} must be a YAML file: {value(name)}')
    sim_time = {'use_sim_time': value('use_sim_time').lower() == 'true'}
    ndt_overrides = dict(sim_time)
    if value('map_path'):
        ndt_overrides['map.path'] = value('map_path')
    ekf_overrides = dict(sim_time)
    if value('vehicle_odometry_input_topic'):
        ekf_overrides['odom0'] = value('vehicle_odometry_input_topic')

    nodes = [
        # 参照: autoware_core_localization.launch.xml から読み込まれる
        # autoware_ndt_scan_matcher/launch/ndt_scan_matcher.launch.xml
        Node(package='ndt_localization', executable='ndt_scan_matcher_node',
             name='ndt_scan_matcher', namespace='localization/pose_estimator', output='screen',
             parameters=[value('ndt_scan_matcher_param_path'), ndt_overrides],
             remappings=[('points_raw', value('lidar_input_topic')),
                         ('ekf_odom', '/localization/kinematic_state'),
                         ('ndt_pose', '/localization/pose_estimator/pose'),
                         ('ndt_pose_with_covariance', '/localization/pose_estimator/pose_with_covariance')]),
        # 参照: autoware_ekf_localizer/launch/ekf_localizer.launch.xml（output_odom_name は
        # /localization/kinematic_state）。初期姿勢は Autoware の /initialpose3d ではなく、RViz の /initialpose を直接受ける
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_localizer', namespace='localization/pose_twist_fusion_filter', output='screen',
             parameters=[value('ekf_localizer_param_path'), ekf_overrides],
             remappings=[('odometry/filtered', '/localization/kinematic_state'),
                         ('set_pose', '/initialpose')]),
    ]
    if value('rviz').lower() == 'true':
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='localization_rviz',
            arguments=['-d', value('rviz_config')], parameters=[sim_time], output='screen'))
    return nodes


def generate_launch_description():
    """launch 引数を宣言し、ノードの生成を OpaqueFunction に任せた LaunchDescription を返す。

    入力: なし（ros2 launch から呼ばれる）。
    出力: LaunchDescription。
    """
    share = get_package_share_directory('ndt_localization')
    config = os.path.join(share, 'config')
    args = [
        # 引数名は、autoware_core_localization.launch.xml に同じものがあればそれに合わせている
        DeclareLaunchArgument('lidar_input_topic', default_value='/surestar_points'),
        DeclareLaunchArgument('vehicle_odometry_input_topic', default_value='',
                              description='Overrides odom0 in ekf_localizer_param_path'),
        DeclareLaunchArgument('ndt_scan_matcher_param_path',
                              default_value=os.path.join(config, 'ndt_scan_matcher.param.yaml')),
        DeclareLaunchArgument('ekf_localizer_param_path',
                              default_value=os.path.join(config, 'ekf_localizer.param.yaml')),
        DeclareLaunchArgument('map_path', default_value='',
                              description='Overrides map.path in ndt_scan_matcher_param_path'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'localization.rviz')),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=create_nodes)])
