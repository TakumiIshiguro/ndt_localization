"""Autoware Core の自己位置推定の launch と同じ構成で、固定地図上の自己位置推定を起動する。

参照元: autoware_core (commit e0a8ab84)
  localization/autoware_core_localization/launch/autoware_core_localization.launch.xml

  名前空間 / ノード                                       | autoware_core の対応
  --------------------------------------------------------+---------------------------------------------
  /sensing/vehicle_velocity_converter                      | autoware_vehicle_velocity_converter（入力は Odometry）
  /sensing/imu/imu_corrector                               | autoware_imu_corrector（Universe。Core にはない）
  /localization/twist_estimator/gyro_odometer              | autoware_gyro_odometer（Core の launch では relay の場所）
  /localization/pose_estimator/ndt_scan_matcher            | autoware_ndt_scan_matcher（名前空間も同じ）
  /localization/pose_twist_fusion_filter/ekf_localizer     | autoware_ekf_localizer（TF は map->odom）
  /localization/util/pose_initializer                      | autoware_pose_initializer（簡易版）
  /localization/util/odometry_integrator                   | （本パッケージ独自）odom->base_link と /odometry/filtered

  /odom（車輪）─▶ vehicle_velocity_converter ─┐
  /imu/data ─▶ imu_corrector ────────────────▶ gyro_odometer ─▶ /localization/twist_estimator/twist_with_covariance
                                                                   │                          │
                                       odometry_integrator ◀───────┘                          ▼
                                         ├ TF odom->base_link            /surestar_points ─▶ ndt_scan_matcher ─▶ ekf_localizer
                                         └ /odometry/filtered（Nav2）           ▲ biased_pose_with_covariance ◀───┤
                                                                                                   ├ TF map->odom
                                                                                                   └ /localization/kinematic_state
  /initialpose（RViz）、user_defined_initial_pose ─▶ pose_initializer ─▶ /initialpose3d、EKF の trigger_node

Autoware との違い: voxel_grid_downsample_filter ノードがない（ndt_scan_matcher の中で前処理する）、
map_loader ノードがない（ndt_scan_matcher が PCD を自分で読む）、stop_filter / twist2accel がない、
pose_initializer は起動時の姿勢と RViz の姿勢だけを扱う（z/roll/pitch は与えた値のまま。map_height_fitter がない）、
NDT の trigger_node がない（常に有効）、EKF は map->base_link ではなく map->odom を配信する
（odom->base_link は odometry_integrator が出す。Nav2 が odom フレームを使うため）。
base_link -> LiDAR / IMU の TF は、ロボット本体か bag から与える。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAM_PATH_ARGS = (
    ('vehicle_velocity_converter_param_path', 'vehicle_velocity_converter.param.yaml'),
    ('imu_corrector_param_path', 'imu_corrector.param.yaml'),
    ('gyro_odometer_param_path', 'gyro_odometer.param.yaml'),
    ('odometry_integrator_param_path', 'odometry_integrator.param.yaml'),
    ('ndt_scan_matcher_param_path', 'ndt_scan_matcher.param.yaml'),
    ('ekf_localizer_param_path', 'ekf_localizer.param.yaml'),
    ('pose_initializer_param_path', 'pose_initializer.param.yaml'),
)


def create_nodes(context):
    """launch 引数の値を確定させてから、起動するノードの一覧を作る（OpaqueFunction から呼ばれる）。

    入力: context  launch のコンテキスト。launch 引数の値を取り出すのに使う。
    出力: 起動する Node のリスト。
    例外: パラメータファイルのパスが YAML ファイルでない場合、initial_pose が 7 要素の配列でない場合は RuntimeError。
    """

    def value(name):
        """launch 引数 name の値を文字列で返す。"""
        return LaunchConfiguration(name).perform(context)

    for name, _ in PARAM_PATH_ARGS:
        if not os.path.isfile(value(name)):
            raise RuntimeError(f'{name} must be a YAML file: {value(name)}')
    sim_time = {'use_sim_time': value('use_sim_time').lower() == 'true'}
    ndt_overrides = dict(sim_time)
    if value('map_path'):
        ndt_overrides['map.path'] = value('map_path')
    # 参照: autoware_core_localization.launch.xml の initial_pose 引数（7 要素なら user_defined_initial_pose を有効にする）
    pose_initializer_overrides = dict(sim_time)
    if value('initial_pose'):
        initial_pose = yaml.safe_load(value('initial_pose'))
        if not isinstance(initial_pose, list) or len(initial_pose) != 7:
            raise RuntimeError(f'initial_pose must be [x, y, z, qx, qy, qz, qw]: {value("initial_pose")}')
        pose_initializer_overrides['user_defined_initial_pose.enable'] = True
        pose_initializer_overrides['user_defined_initial_pose.pose'] = [float(v) for v in initial_pose]

    def param(name):
        """パラメータファイルの launch 引数 name の値を返す。"""
        return value(name + '_param_path')

    nodes = [
        # 参照: autoware_vehicle_velocity_converter/launch/vehicle_velocity_converter.launch.xml
        # （autoware_core_sensing.launch.xml から /sensing の下で起動される）
        Node(package='ndt_localization', executable='vehicle_velocity_converter_node',
             name='vehicle_velocity_converter', namespace='sensing', output='screen',
             parameters=[param('vehicle_velocity_converter'), sim_time],
             remappings=[('odometry', value('vehicle_odometry_input_topic')),
                         ('twist_with_covariance', '/sensing/vehicle_velocity_converter/twist_with_covariance')]),
        # 参照: autoware_universe の autoware_imu_corrector/launch/imu_corrector.launch.xml。名前空間と出力のトピックは
        # Universe の sensing の構成（/sensing/imu の下、出力は imu_data）に合わせる。IMU の共分散が 0 だと EKF が
        # 角速度を完全に信じて速度を棄却するので、Core にはないが入れている
        Node(package='ndt_localization', executable='imu_corrector_node',
             name='imu_corrector', namespace='sensing/imu', output='screen',
             parameters=[param('imu_corrector'), sim_time],
             remappings=[('input', value('imu_input_topic')),
                         ('output', '/sensing/imu/imu_data')]),
        # 参照: autoware_gyro_odometer/launch/gyro_odometer.launch.xml。Core の launch の twist_estimator の場所に置き、
        # 出力を /localization/twist_estimator/twist_with_covariance にする（Universe と同じ）
        Node(package='ndt_localization', executable='gyro_odometer_node',
             name='gyro_odometer', namespace='localization/twist_estimator', output='screen',
             parameters=[param('gyro_odometer'), sim_time],
             remappings=[('vehicle/twist_with_covariance', '/sensing/vehicle_velocity_converter/twist_with_covariance'),
                         ('imu', '/sensing/imu/imu_data'),
                         ('twist_raw', 'gyro_twist_raw'),
                         ('twist_with_covariance_raw', 'gyro_twist_with_covariance_raw'),
                         ('twist', 'gyro_twist'),
                         ('twist_with_covariance', '/localization/twist_estimator/twist_with_covariance')]),
        # (本パッケージ独自) Nav2 用の odom->base_link と /odometry/filtered
        Node(package='ndt_localization', executable='odometry_integrator_node',
             name='odometry_integrator', namespace='localization/util', output='screen',
             parameters=[param('odometry_integrator'), sim_time],
             remappings=[('twist_with_covariance', '/localization/twist_estimator/twist_with_covariance'),
                         ('odometry', '/odometry/filtered')]),
        # 参照: autoware_ndt_scan_matcher/launch/ndt_scan_matcher.launch.xml
        Node(package='ndt_localization', executable='ndt_scan_matcher_node',
             name='ndt_scan_matcher', namespace='localization/pose_estimator', output='screen',
             parameters=[param('ndt_scan_matcher'), ndt_overrides],
             remappings=[('points_raw', value('lidar_input_topic')),
                         ('ekf_pose_with_covariance',
                          '/localization/pose_twist_fusion_filter/biased_pose_with_covariance'),
                         ('ndt_pose', '/localization/pose_estimator/pose'),
                         ('ndt_pose_with_covariance', '/localization/pose_estimator/pose_with_covariance')]),
        # 参照: autoware_ekf_localizer/launch/ekf_localizer.launch.xml と、autoware_core_localization.launch.xml が
        # 渡す引数。stop_filter がないので、ekf_odom を直接 /localization/kinematic_state にする
        Node(package='ndt_localization', executable='ekf_localizer_node',
             name='ekf_localizer', namespace='localization/pose_twist_fusion_filter', output='screen',
             parameters=[param('ekf_localizer'), sim_time],
             remappings=[('in_pose_with_covariance', '/localization/pose_estimator/pose_with_covariance'),
                         ('in_twist_with_covariance', '/localization/twist_estimator/twist_with_covariance'),
                         ('initialpose', '/initialpose3d'),
                         ('trigger_node_srv', 'trigger_node'),
                         ('ekf_odom', '/localization/kinematic_state'),
                         ('ekf_pose', 'pose'),
                         ('ekf_pose_with_covariance', '/localization/pose_with_covariance'),
                         ('ekf_biased_pose', 'biased_pose'),
                         ('ekf_biased_pose_with_covariance', 'biased_pose_with_covariance'),
                         ('ekf_twist', 'twist'),
                         ('ekf_twist_with_covariance', 'twist_with_covariance')]),
        # 参照: autoware_pose_initializer/launch/pose_initializer.launch.xml（pose_reset と ekf_trigger_node の remap）。
        # RViz の /initialpose を直接受ける（Universe の initial_pose_adaptor の役割）
        Node(package='ndt_localization', executable='pose_initializer_node',
             name='pose_initializer', namespace='localization/util', output='screen',
             parameters=[param('pose_initializer'), pose_initializer_overrides],
             remappings=[('initialpose', '/initialpose'),
                         ('pose_reset', '/initialpose3d'),
                         ('ekf_trigger_node', '/localization/pose_twist_fusion_filter/trigger_node')]),
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
        DeclareLaunchArgument('vehicle_odometry_input_topic', default_value='/odom',
                              description='Wheel odometry (nav_msgs/Odometry); its twist is used'),
        DeclareLaunchArgument('imu_input_topic', default_value='/imu/data'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='Startup pose [x, y, z, qx, qy, qz, qw] in map; overrides '
                                          'user_defined_initial_pose in pose_initializer_param_path'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='Overrides map.path in ndt_scan_matcher_param_path'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'localization.rviz')),
    ]
    args += [DeclareLaunchArgument(name, default_value=os.path.join(config, file_name))
             for name, file_name in PARAM_PATH_ARGS]
    return LaunchDescription(args + [OpaqueFunction(function=create_nodes)])
