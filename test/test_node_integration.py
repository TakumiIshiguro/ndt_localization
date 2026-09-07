"""Isolated ROS smoke test: map latching, scan/odom/TF path, LOST and reinitialization.

Run after building: python3 test/test_node_integration.py --executable <localization_node>
This script publishes synthetic data only in its own ROS domain.
"""
import argparse
import os
import random
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


def run(executable):
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(130, 220))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='wheel-lidar-integration-') as temp:
        os.environ['ROS_LOG_DIR'] = temp
        rng = np.random.default_rng(42)
        centers = rng.uniform([-7, -7, -2], [7, 7, 2], (100, 3))
        points = (centers[:, None, :] + rng.normal(0, 0.18, (100, 80, 3))).reshape(-1, 3).astype('<f4')
        map_file = Path(temp) / 'map.pcd'
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
        map_file.write_bytes(header.encode() + points.tobytes())
        log = open(Path(temp) / 'node.log', 'w+')
        command = [executable, '--ros-args', '-p', f'map_path:={map_file}',
                   '-p', 'cloud_topic:=/test/cloud', '-p', 'odom_topic:=/test/odom',
                   '-p', 'map_voxel_size:=0.1', '-p', 'scan_voxel_size:=0.1',
                   '-p', 'scan_min_range:=0.01', '-p', 'ndt_max_iterations:=80',
                   '-p', 'ndt_transformation_epsilon:=0.001', '-p', 'max_scan_age:=5.0',
                   '-p', 'stale_timeout:=5.0', '-p', 'failures_to_lost:=3']
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('wheel_localization_integration_probe')
        received = {'map': None, 'pose': None, 'status': None, 'tf': None}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        subscriptions = [
            node.create_subscription(PointCloud2, '/localization/map', lambda m: received.update(map=m), latched),
            node.create_subscription(PoseWithCovarianceStamped, '/localization/pose', lambda m: received.update(pose=m), 10),
            node.create_subscription(DiagnosticArray, '/localization/status', lambda m: received.update(status=m), latched),
        ]

        def tf_callback(msg):
            for transform in msg.transforms:
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    received['tf'] = transform
        subscriptions.append(node.create_subscription(TFMessage, '/tf', tf_callback, 100))
        cloud_pub = node.create_publisher(PointCloud2, '/test/cloud', 10)
        odom_pub = node.create_publisher(Odometry, '/test/odom', 100)
        initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        static = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'test_lidar'
        transform.transform.translation.z = 0.4
        transform.transform.rotation.w = 1.0
        static.sendTransform(transform)

        def spin_until(predicate, timeout=15.0, tick=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, 'localizer exited'
                if tick:
                    tick()
                rclpy.spin_once(node, timeout_sec=0.03)
                if predicate():
                    return
            raise AssertionError('Timed out; latest status: ' + str(received['status']))

        def state():
            if not received['status']:
                return ''
            return dict((v.key, v.value) for v in received['status'].status[0].values).get('state', '')

        def initial():
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            # Zero stamp explicitly allows the next synthetic scan.
            message.pose.pose.orientation.w = 1.0
            initial_pub.publish(message)

        last_sent = [0.0]

        def send_scan(empty=False):
            if time.monotonic() - last_sent[0] < 0.15:
                return
            last_sent[0] = time.monotonic()
            current = node.get_clock().now().nanoseconds
            stamp_ns = current - 10_000_000
            # The cloud reference time is bracketed, and never extrapolated.
            for offset in (-20_000_000, 0):
                odom = Odometry()
                odom.header.stamp = Time(nanoseconds=current + offset).to_msg()
                odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
            msg = PointCloud2()
            msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            msg.header.frame_id = 'test_lidar'
            lidar_points = points - np.array([0, 0, 0.4], dtype='<f4')
            if empty:
                lidar_points = lidar_points[:0]
            msg.height = 1; msg.width = len(lidar_points)
            msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate(('x', 'y', 'z'))]
            msg.point_step = 12; msg.row_step = msg.width * 12
            msg.is_dense = True; msg.data = lidar_points.astype('<f4').tobytes()
            cloud_pub.publish(msg)

        try:
            spin_until(lambda: received['map'] is not None and state() == 'WAITING_INITIAL_POSE')
            assert received['map'].width > 100
            assert received['pose'] is None, 'pose published before initialization'
            # Late subscriber must receive the already-published map.
            received['late_map'] = None
            subscriptions.append(node.create_subscription(PointCloud2, '/localization/map',
                lambda m: received.update(late_map=m), latched))
            spin_until(lambda: received['late_map'] is not None)
            spin_until(lambda: initial_pub.get_subscription_count() > 0)
            initial()
            spin_until(lambda: state() == 'TRACKING' and received['tf'] is not None, tick=send_scan)
            pose = received['pose'].pose.pose.position
            assert np.linalg.norm([pose.x, pose.y, pose.z]) < 0.15, pose
            correction = received['tf'].transform.translation
            held = np.array([correction.x, correction.y, correction.z])
            spin_until(lambda: state() == 'LOST', tick=lambda: send_scan(empty=True))
            correction = received['tf'].transform.translation
            assert np.linalg.norm(np.array([correction.x, correction.y, correction.z]) - held) < 0.02
            initial()
            # Wait for the new epoch before checking TRACKING again.
            spin_until(lambda: state() == 'INITIALIZING')
            spin_until(lambda: state() == 'TRACKING', tick=send_scan)
            print('PASS: latched map, initial pose, timestamp interpolation, sensor TF, NDT pose, held correction, LOST, reinitialization')
        finally:
            node.destroy_node(); rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
            log.seek(0)
            print(log.read())
            log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    run(parser.parse_args().executable)
