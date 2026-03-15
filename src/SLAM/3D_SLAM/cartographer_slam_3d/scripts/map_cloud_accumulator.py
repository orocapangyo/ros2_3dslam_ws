#!/usr/bin/env python3
"""
Cartographer 3D 맵 포인트클라우드 누적기

/scan_matched_points2 (매 스캔) → 누적 → /map_cloud (전체 맵)
매 5초마다 누적된 전체 맵을 발행.
voxel grid로 다운샘플링하여 메모리 절약.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import struct


class MapCloudAccumulator(Node):
    def __init__(self):
        super().__init__('map_cloud_accumulator')
        self.declare_parameter('voxel_size', 0.1)
        self.voxel_size = self.get_parameter('voxel_size').value

        self.sub_ = self.create_subscription(
            PointCloud2, '/scan_matched_points2', self.callback, 10)

        # Transient Local로 발행 → RViz가 나중에 구독해도 최신 맵 받음
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ = self.create_publisher(PointCloud2, '/map_cloud', qos)

        self.declare_parameter('warmup_sec', 10.0)
        self.warmup_sec = self.get_parameter('warmup_sec').value

        self.all_points = {}  # voxel key → (x, y, z)
        self.timer_ = self.create_timer(5.0, self.publish_map)
        self.msg_count = 0
        self.start_time = None
        self.warmed_up = False
        self.get_logger().info(
            f'Map cloud accumulator started (voxel={self.voxel_size}m, '
            f'warmup={self.warmup_sec}s)')

    def callback(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < self.warmup_sec:
            if not self.warmed_up:
                return
        elif not self.warmed_up:
            self.warmed_up = True
            self.get_logger().info(
                f'Warmup complete ({self.warmup_sec}s). Accumulating map.')

        points = self.parse_pointcloud2(msg)
        if points is None:
            return
        vs = self.voxel_size
        for p in points:
            key = (int(p[0] / vs), int(p[1] / vs), int(p[2] / vs))
            if key not in self.all_points:
                self.all_points[key] = (p[0], p[1], p[2])
        self.msg_count += 1

    def publish_map(self):
        if not self.all_points:
            return
        points = list(self.all_points.values())
        msg = self.create_pointcloud2(points)
        self.pub_.publish(msg)
        self.get_logger().info(
            f'Published map_cloud: {len(points)} points '
            f'(from {self.msg_count} scans)')

    def parse_pointcloud2(self, msg):
        if msg.width == 0:
            return None
        fmt = 'fff'  # x, y, z as float32
        step = msg.point_step
        points = []
        for i in range(msg.width * msg.height):
            offset = i * step
            if offset + 12 > len(msg.data):
                break
            x, y, z = struct.unpack_from('fff', msg.data, offset)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append((x, y, z))
        return points

    def create_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        data = bytearray()
        for p in points:
            data += struct.pack('fff', p[0], p[1], p[2])
        msg.data = data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MapCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
