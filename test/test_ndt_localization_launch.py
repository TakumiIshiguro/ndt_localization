"""ndt_localization.launch.py を合成データで通しで動かす結合テスト（専用の ROS ドメインで実行）。

autoware_core の localization/autoware_ndt_scan_matcher/test/test_ndt_scan_matcher_launch.py に倣っているが、
launch のノード（vehicle_velocity_converter、gyro_odometer、odometry_integrator、ndt_scan_matcher、
ekf_localizer、pose_initializer）をまとめて動かす。

ロボットが合成 3D 地図の中を直進する。テストは、ロボット本体が出すはずのもの（車輪オドメトリ /odom、
IMU /imu/data、静的 TF base_link->lidar、LiDAR 点群）と、RViz 相当の /initialpose を 1 回配信する。
そして TF の map->odom（ekf_localizer）と odom->base_link（odometry_integrator）をつないだ map->base_link と、
/localization/kinematic_state が、真のロボット姿勢に収束して追跡を続けるかを確認する。

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
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

# 真値: ロボットは map 上の START から、その向きのまま一定速度で直進する
START = (1.0, 0.5, 0.1)  # x, y, yaw
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
        # 合成地図は実際の地図より NVTL が低く出るので閾値を下げる（test_ndt_omp を参照）
        p['score_estimation']['converged_param_nearest_voxel_transformation_likelihood'] = 0.5
        ndt_params = temp / 'ndt.yaml'
        ndt_params.write_text(yaml.safe_dump(params))
        # 起動時の姿勢を使わない。こうすると EKF は /initialpose を受け取るまで動かず、
        # 収束したら /initialpose の姿勢から収束したと言える
        initializer = yaml.safe_load((share / 'config' / 'pose_initializer.param.yaml').read_text())
        initializer['/**']['ros__parameters']['user_defined_initial_pose']['enable'] = False
        initializer_params = temp / 'pose_initializer.yaml'
        initializer_params.write_text(yaml.safe_dump(initializer))
        log = open(temp / 'launch.log', 'w+')
        process = subprocess.Popen(
            ['ros2', 'launch', 'ndt_localization', 'ndt_localization.launch.py', 'rviz:=false',
             f'ndt_scan_matcher_param_path:={ndt_params}', f'pose_initializer_param_path:={initializer_params}',
             'lidar_input_topic:=/test/cloud', 'vehicle_odometry_input_topic:=/test/odom',
             'imu_input_topic:=/test/imu'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        rclpy.init()
        node = rclpy.create_node('ndt_localization_integration_probe')
        received = {'map': None, 'ndt_pose': None, 'map_to_odom': None, 'odom_to_base': None, 'odometry': None,
                    'kinematic': None, 'ndt_ok': False}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        def tf_callback(msg):
            """/tf の購読コールバック。map->odom と odom->base_link の変換を最新値として保存する。"""
            for t in msg.transforms:
                if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                    received['map_to_odom'] = t
                if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
                    received['odom_to_base'] = t

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
            node.create_subscription(Odometry, '/odometry/filtered', lambda m: received.update(odometry=m), 10),
            node.create_subscription(TFMessage, '/tf', tf_callback, 100),
            node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics_callback, 100),
        ]
        cloud_pub = node.create_publisher(PointCloud2, '/test/cloud', 10)
        odom_pub = node.create_publisher(Odometry, '/test/odom', 100)
        imu_pub = node.create_publisher(Imu, '/test/imu', 100)
        initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        ekf_trigger = node.create_client(SetBool, '/localization/pose_twist_fusion_filter/trigger_node')
        static = StaticTransformBroadcaster(node)
        lidar = TransformStamped()
        lidar.header.frame_id = 'base_link'
        lidar.child_frame_id = 'test_lidar'
        lidar.transform.translation.z = LIDAR_Z
        lidar.transform.rotation.w = 1.0
        static.sendTransform(lidar)

        start = node.get_clock().now().nanoseconds * 1e-9
        last = {'odom': 0.0, 'cloud': 0.0}
        mx, my, myaw = START

        def truth_pose(stamp_sec):
            """時刻 stamp_sec [s]（ROS 時刻）での真のロボット姿勢 (x, y, yaw)（map 座標系）を返す。"""
            d = SPEED * (stamp_sec - start)
            return mx + math.cos(myaw) * d, my + math.sin(myaw) * d, myaw

        def tick():
            """ロボット本体の代わりに、必要な周期でデータを配信する。

            入力: なし（現在時刻と真の姿勢から作る）
            出力: 50 Hz で車輪オドメトリ /test/odom（速度だけ。icart のドライバと同じく共分散は 0）と
                  IMU /test/imu（角速度 0）、10 Hz で LiDAR 点群（真の姿勢から見た地図の点、MAX_RANGE 以内）を配信する。
            """
            now_mono = time.monotonic()
            now = node.get_clock().now()
            stamp = now.to_msg()
            bx, by, byaw = truth_pose(now.nanoseconds * 1e-9)
            if now_mono - last['odom'] >= 0.02:
                last['odom'] = now_mono
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.orientation.w = 1.0
                odom.twist.twist.linear.x = SPEED
                odom_pub.publish(odom)
                imu = Imu()
                imu.header.stamp = stamp
                imu.header.frame_id = 'base_link'
                imu.orientation.w = 1.0
                imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = 1e-4
                imu.angular_velocity_covariance[8] = 1e-4
                imu_pub.publish(imu)
            if now_mono - last['cloud'] >= 0.1:
                last['cloud'] = now_mono
                # 地図の点を、真の姿勢にある LiDAR の座標系で表す
                c, s = math.cos(byaw), math.sin(byaw)
                base = np.array([bx, by, 0.0])
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

        def yaw_of(q):
            """クォータニオン q の yaw 角 [rad] を返す。"""
            return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        def pose_error(x, y, yaw, stamp):
            """map 座標系の姿勢 (x, y, yaw) と、時刻 stamp での真の姿勢との誤差 (位置 [m], yaw [rad]) を返す。"""
            tx, ty, tyaw = truth_pose(stamp.sec + stamp.nanosec * 1e-9)
            return math.hypot(x - tx, y - ty), abs(math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw)))

        def tf_error():
            """最後に受信した map->odom と odom->base_link をつないだ map->base_link の、真値との誤差を返す。

            入力: なし
            出力: (位置の誤差 [m], yaw の誤差 [rad])。どちらかをまだ受信していなければ (inf, inf)。
                  真値は odom->base_link の時刻のもの。
            """
            mo, ob = received['map_to_odom'], received['odom_to_base']
            if mo is None or ob is None:
                return float('inf'), float('inf')
            oyaw = yaw_of(mo.transform.rotation)
            c, s = math.cos(oyaw), math.sin(oyaw)
            bx, by = ob.transform.translation.x, ob.transform.translation.y
            return pose_error(mo.transform.translation.x + c * bx - s * by, mo.transform.translation.y + s * bx + c * by,
                              oyaw + yaw_of(ob.transform.rotation), ob.header.stamp)

        def kinematic_error():
            """最後に受信した /localization/kinematic_state の、その時刻の真値との誤差 (位置 [m], yaw [rad]) を返す。"""
            k = received['kinematic']
            if k is None:
                return float('inf'), float('inf')
            return pose_error(k.pose.pose.position.x, k.pose.pose.position.y, yaw_of(k.pose.pose.orientation),
                              k.header.stamp)

        try:
            spin_until(lambda: received['map'] is not None, 'latched map', timeout=60.0)
            assert received['map'].width > 1000
            spin_until(lambda: received['odom_to_base'] is not None and received['odometry'] is not None,
                       'odometry_integrator output (odom->base_link, /odometry/filtered)')
            spin_until(lambda: initial_pub.get_subscription_count() > 0 and ekf_trigger.service_is_ready(),
                       'pose_initializer subscribing /initialpose and the EKF trigger service')
            # pose_initializer の起動時の姿勢を無効にしたので、/initialpose を受け取るまで EKF は何も出さない
            assert received['kinematic'] is None and received['map_to_odom'] is None
            def publish_initial_pose():
                """RViz 相当の 2D 姿勢を、今の真値から 0.4 m・0.08 rad ずらして /initialpose に配信する。"""
                now = node.get_clock().now()
                tx, ty, tyaw = truth_pose(now.nanoseconds * 1e-9)
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = 'map'
                initial.header.stamp = now.to_msg()
                initial.pose.pose.position.x = tx + 0.3
                initial.pose.pose.position.y = ty - 0.25
                q = yaw_quaternion(tyaw + 0.08)
                (initial.pose.pose.orientation.x, initial.pose.pose.orientation.y,
                 initial.pose.pose.orientation.z, initial.pose.pose.orientation.w) = q
                # RViz の 2D Pose Estimate と同じ共分散（x/y: 0.25、yaw: 0.0685、他は 0）
                initial.pose.covariance[0] = initial.pose.covariance[7] = 0.25
                initial.pose.covariance[35] = 0.06853891945200942
                initial_pub.publish(initial)

            # 接続の直後は 1 回目の /initialpose が届かないことがあるので、EKF が出力を始めるまで送り直す
            # （RViz で姿勢を指定し直すのと同じ）
            for attempt in range(3):
                publish_initial_pose()
                try:
                    spin_until(lambda: received['kinematic'] is not None, 'EKF output after /initialpose', timeout=3.0)
                    break
                except AssertionError:
                    if attempt == 2:
                        raise
            spin_until(lambda: received['ndt_pose'] is not None and received['ndt_ok'], 'converged NDT pose')
            spin_until(lambda: tf_error()[0] < 0.1 and tf_error()[1] < 0.02 and kinematic_error()[0] < 0.1
                       and kinematic_error()[1] < 0.02, 'map->odom->base_link and kinematic_state to converge to the truth')
            # そのまま 5 秒走らせ、その間ずっと追跡が続くことを確認する
            converged = node.get_clock().now().nanoseconds * 1e-9
            received['ndt_pose'] = None
            hold_until = time.monotonic() + 5.0
            while time.monotonic() < hold_until:
                assert process.poll() is None, 'launch exited'
                tick()
                time.sleep(0.005)
                assert tf_error()[0] < 0.1, f'lost track: map->base_link error {tf_error()}'
                assert kinematic_error()[0] < 0.1, f'lost track: kinematic_state error {kinematic_error()}'
            assert received['ndt_pose'] is not None, 'NDT stopped publishing'
            assert SPEED * (node.get_clock().now().nanoseconds * 1e-9 - converged) > 2.0
            assert received['kinematic'].header.frame_id == 'map'
            assert received['kinematic'].child_frame_id == 'base_link'
            assert received['odometry'].header.frame_id == 'odom'
            assert received['odometry'].child_frame_id == 'base_link'
            assert abs(received['odometry'].twist.twist.linear.x - SPEED) < 1e-6
            pos_err, yaw_err = tf_error()
            print(f'PASS: map, odometry, initial pose, NDT pose, EKF fusion, 5 s tracking; map->odom->base_link error '
                  f'{pos_err:.3f} m / {yaw_err:.4f} rad after {node.get_clock().now().nanoseconds * 1e-9 - start:.1f} s')
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
