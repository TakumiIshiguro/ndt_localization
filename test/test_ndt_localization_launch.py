"""ndt_localization.launch.py を合成データで通しで動かす結合テスト（専用の ROS ドメインで実行）。

autoware_core の localization/autoware_ndt_scan_matcher/test/test_ndt_scan_matcher_launch.py に倣っているが、
2 つのノード（ndt_scan_matcher、ekf_localizer）をまとめて動かす。

ロボットが合成 3D 地図の中を直進する。テストは、ロボット本体と odom 用 EKF が出すはずのもの
（/odometry/filtered、TF odom->base_link、静的 TF base_link->lidar、LiDAR 点群）と、RViz 相当の
/initialpose を 1 回配信する。そして ndt_scan_matcher + ekf_localizer（robot_localization）が、
真の map->odom を復元できるかを確認する。

ビルドして install/setup.bash を読み込んでから実行する:
    python3 test/test_ndt_localization_launch.py
"""
import math
import os
import random
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# 真値: map->odom はロボットの出発姿勢。ロボットはその後 odom 座標系で x 方向に直進する
MAP_TO_ODOM = (1.0, 0.5, 0.1)  # x, y, yaw
SPEED = 0.5
LIDAR_Z = 0.4
MAX_RANGE = 18.0


def yaw_quaternion(yaw):
    """yaw 角だけを持つクォータニオンを作る。

    入力: yaw  yaw 角 [rad]
    出力: (x, y, z, w) のタプル
    """
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def make_map(rng):
    """テスト用の合成 3D 地図を作る。

    x: -15〜25 m、y: ±15 m、z: -1〜3 m の範囲に 400 個の塊を置き、各塊に σ=0.15 m で 40 点を散らす。
    入力: rng  numpy の乱数生成器（種を固定すれば毎回同じ地図になる）
    出力: (16000, 3) の float32 配列（map 座標系）
    """
    centers = rng.uniform([-15, -15, -1], [25, 15, 3], (400, 3))
    return (centers[:, None, :] + rng.normal(0, 0.15, (400, 40, 3))).reshape(-1, 3).astype('<f4')


def write_pcd(points, path):
    """点群をバイナリ形式の PCD ファイルに書き出す。

    入力: points  (N, 3) の float32 配列、path  書き出し先の Path
    出力: なし（ファイルを書く）
    """
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode() + points.tobytes())


def run():
    """結合テストの本体。launch を起動し、合成データを流して結果を検証する。

    入力: なし
    出力: 成功時は "PASS: ..." を表示する。失敗時は AssertionError を投げる。
          どちらの場合も最後に launch のログの末尾を表示する。
    """
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(130, 220))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    share = Path(get_package_share_directory('ndt_localization'))
    with tempfile.TemporaryDirectory(prefix='ndt-localization-integration-') as temp:
        temp = Path(temp)
        os.environ['ROS_LOG_DIR'] = str(temp)
        rng = np.random.default_rng(42)
        points = make_map(rng)
        write_pcd(points, temp / 'map.pcd')
        params = yaml.safe_load((share / 'config' / 'ndt_scan_matcher.param.yaml').read_text())
        p = params['/**']['ros__parameters']
        p['map']['path'] = str(temp / 'map.pcd')
        p['map']['voxel_size'] = 0.1
        # 合成地図は実際の地図より NVTL が低く出るので閾値を下げる（test_ndt_omp を参照）
        p['score_estimation']['converged_param_nearest_voxel_transformation_likelihood'] = 0.5
        ndt_params = temp / 'ndt.yaml'
        ndt_params.write_text(yaml.safe_dump(params))
        # EKF の起動時の姿勢を地図の外に置く。こうすると /initialpose を受け取る前に NDT が収束することはなく、
        # 収束したら初期姿勢から収束したと言える
        ekf = yaml.safe_load((share / 'config' / 'ekf_localizer.param.yaml').read_text())
        ekf['/**']['ros__parameters']['initial_state'][0:2] = [200.0, 200.0]
        ekf_params = temp / 'ekf.yaml'
        ekf_params.write_text(yaml.safe_dump(ekf))
        log = open(temp / 'launch.log', 'w+')
        process = subprocess.Popen(
            ['ros2', 'launch', 'ndt_localization', 'ndt_localization.launch.py', 'rviz:=false',
             f'ndt_scan_matcher_param_path:={ndt_params}', f'ekf_localizer_param_path:={ekf_params}',
             'lidar_input_topic:=/test/cloud'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        rclpy.init()
        node = rclpy.create_node('ndt_localization_integration_probe')
        received = {'map': None, 'ndt_pose': None, 'map_to_odom': None, 'kinematic': None, 'ndt_ok': False}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        def tf_callback(msg):
            """/tf の購読コールバック。map->odom の変換だけを最新値として保存する。"""
            for t in msg.transforms:
                if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                    received['map_to_odom'] = t

        def diagnostics_callback(msg):
            """/diagnostics の購読コールバック。NDT の scan_matching_status を保存し、一度でも OK になったら記録する。"""
            for status in msg.status:
                if status.name.endswith('scan_matching_status'):
                    received['ndt_status'] = status
                    # rclpy ではメッセージの byte 型が bytes になるので、両方と比べる
                    received['ndt_ok'] |= status.level in (0, b'\x00')

        subscriptions = [  # noqa: F841 (kept alive)
            node.create_subscription(PointCloud2, '/localization/pose_estimator/debug/loaded_pointcloud_map', lambda m: received.update(map=m), latched),
            node.create_subscription(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance',
                                     lambda m: received.update(ndt_pose=m), 10),
            node.create_subscription(Odometry, '/localization/kinematic_state',
                                     lambda m: received.update(kinematic=m), 10),
            node.create_subscription(TFMessage, '/tf', tf_callback, 100),
            node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics_callback, 100),
        ]
        cloud_pub = node.create_publisher(PointCloud2, '/test/cloud', 10)
        odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 100)
        initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        tf_broadcaster = TransformBroadcaster(node)
        static = StaticTransformBroadcaster(node)
        lidar = TransformStamped()
        lidar.header.frame_id = 'base_link'
        lidar.child_frame_id = 'test_lidar'
        lidar.transform.translation.z = LIDAR_Z
        lidar.transform.rotation.w = 1.0
        static.sendTransform(lidar)

        start = time.monotonic()
        last = {'odom': 0.0, 'cloud': 0.0}
        mx, my, myaw = MAP_TO_ODOM

        def truth_odom_x():
            """現在の真のロボット位置（odom 座標系の x [m]）を返す。開始からの経過時間 × 速度。"""
            return SPEED * (time.monotonic() - start)

        def tick():
            """ロボット本体と odom 用 EKF の代わりに、必要な周期でデータを配信する。

            入力: なし（現在時刻と真の位置から作る）
            出力: 50 Hz で /odometry/filtered と TF odom->base_link、
                  10 Hz で LiDAR 点群（真の姿勢から見た地図の点、MAX_RANGE 以内）を配信する。
            """
            now_mono = time.monotonic()
            stamp = node.get_clock().now().to_msg()
            x = truth_odom_x()
            if now_mono - last['odom'] >= 0.02:
                last['odom'] = now_mono
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = x
                odom.pose.pose.orientation.w = 1.0
                odom.twist.twist.linear.x = SPEED
                for i in range(6):
                    odom.pose.covariance[i * 7] = 0.01
                    odom.twist.covariance[i * 7] = 0.001
                odom_pub.publish(odom)
                t = TransformStamped()
                t.header = odom.header
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x
                t.transform.rotation.w = 1.0
                tf_broadcaster.sendTransform(t)
            if now_mono - last['cloud'] >= 0.1:
                last['cloud'] = now_mono
                # map 上のロボット姿勢 = map->odom * (x, 0, 0)。地図の点を LiDAR 座標系で表す
                c, s = math.cos(myaw), math.sin(myaw)
                base = np.array([mx + c * x, my + s * x, 0.0])
                rel = points - base
                local = np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1],
                                  rel[:, 2] - LIDAR_Z], axis=1).astype('<f4')
                local = local[np.linalg.norm(local, axis=1) < MAX_RANGE]
                msg = PointCloud2()
                msg.header.stamp = stamp
                msg.header.frame_id = 'test_lidar'
                msg.height, msg.width = 1, len(local)
                msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                              for i, n in enumerate('xyz')]
                msg.point_step, msg.row_step = 12, 12 * len(local)
                msg.is_dense = True
                msg.data = local.tobytes()
                cloud_pub.publish(msg)

        # 受信は別スレッドで行う。点群を作るスレッドで受信すると受信が遅れて、判定に古い値を使ってしまうため
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        def spin_until(predicate, what, timeout=40.0):
            """条件が成り立つまで、データを配信しながら待つ。

            入力: predicate  引数なしで真偽を返す条件、what  タイムアウト時のメッセージに使う説明、
                  timeout  待つ上限 [s]
            出力: なし（条件が成り立てば戻る）
            例外: launch が終了した場合、またはタイムアウトした場合は AssertionError
                  （最後の NDT の診断内容を含める）
            """
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, 'launch exited'
                tick()
                time.sleep(0.005)
                if predicate():
                    return
            status = received.get('ndt_status')
            detail = '' if status is None else f"; last scan_matching_status: {status.message} " + str(
                {v.key: v.value for v in status.values})
            raise AssertionError('Timed out waiting for ' + what + detail)

        def map_to_odom_error():
            """最後に受信した map->odom と真値の誤差を返す。

            入力: なし
            出力: (位置の誤差 [m], yaw の誤差 [rad])。まだ受信していなければ (inf, inf)
            """
            t = received['map_to_odom']
            if t is None:
                return float('inf'), float('inf')
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (math.hypot(t.transform.translation.x - mx, t.transform.translation.y - my),
                    abs(math.atan2(math.sin(yaw - myaw), math.cos(yaw - myaw))))

        try:
            spin_until(lambda: received['map'] is not None, 'latched map', timeout=60.0)
            assert received['map'].width > 1000
            spin_until(lambda: initial_pub.get_subscription_count() > 0, 'ekf_localizer subscribing /initialpose')
            # RViz 相当の 2D 姿勢を、真値から 0.4 m・0.08 rad ずらして与える
            spin_until(lambda: received['kinematic'] is not None, 'EKF output')
            x = truth_odom_x()
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.header.stamp = node.get_clock().now().to_msg()
            initial.pose.pose.position.x = mx + math.cos(myaw) * x + 0.3
            initial.pose.pose.position.y = my + math.sin(myaw) * x - 0.25
            q = yaw_quaternion(myaw + 0.08)
            (initial.pose.pose.orientation.x, initial.pose.pose.orientation.y,
             initial.pose.pose.orientation.z, initial.pose.pose.orientation.w) = q
            # RViz の 2D Pose Estimate と同じ共分散（x/y: 0.25、yaw: 0.0685、他は 0）
            initial.pose.covariance[0] = initial.pose.covariance[7] = 0.25
            initial.pose.covariance[35] = 0.06853891945200942
            initial_pub.publish(initial)
            spin_until(lambda: received['ndt_pose'] is not None and received['ndt_ok'], 'converged NDT pose')
            spin_until(lambda: map_to_odom_error()[0] < 0.1 and map_to_odom_error()[1] < 0.02,
                       'map->odom to converge to the truth')
            # そのまま 5 秒走らせ、その間ずっと追跡が続くことを確認する
            x_converged = truth_odom_x()
            received['ndt_pose'] = None
            hold_until = time.monotonic() + 5.0
            while time.monotonic() < hold_until:
                assert process.poll() is None, 'launch exited'
                tick()
                time.sleep(0.005)
                assert map_to_odom_error()[0] < 0.1, f'lost track: map->odom error {map_to_odom_error()}'
            assert received['ndt_pose'] is not None, 'NDT stopped publishing'
            assert truth_odom_x() - x_converged > 2.0
            assert received['kinematic'].header.frame_id == 'map'
            assert received['kinematic'].child_frame_id == 'base_link'
            pos_err, yaw_err = map_to_odom_error()
            print(f'PASS: map, initial pose, NDT pose, EKF fusion, 5 s tracking; map->odom error '
                  f'{pos_err:.3f} m / {yaw_err:.4f} rad after {time.monotonic() - start:.1f} s')
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, 2)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, 9)
                process.wait(timeout=5)
            log.seek(0)
            print(log.read()[-6000:])
            log.close()


if __name__ == '__main__':
    run()
