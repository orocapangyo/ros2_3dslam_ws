#!/usr/bin/env python3
"""
IMU QoS Relay for Cartographer 3D SLAM (Gazebo)

Gazebo bridge publishes /imu/data with BEST_EFFORT QoS.
Cartographer subscribes with RELIABLE QoS (default) -> no data received.

This relay:
  - Subscribes to /imu/data           (BEST_EFFORT, matches Gazebo bridge)
  - Republishes to /imu/data_reliable  (RELIABLE, matches Cartographer)
  - Filters duplicate timestamps (Cartographer crashes on duplicate times)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuQosRelay(Node):
    def __init__(self):
        super().__init__('imu_qos_relay')
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_ = self.create_subscription(Imu, '/imu/data', self.callback, be_qos)
        self.pub_ = self.create_publisher(Imu, '/imu/data_reliable', 10)
        self.last_stamp_ = 0
        self.get_logger().info('IMU QoS relay started: /imu/data -> /imu/data_reliable')

    def callback(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp > self.last_stamp_:
            self.pub_.publish(msg)
            self.last_stamp_ = stamp


def main(args=None):
    rclpy.init(args=args)
    node = ImuQosRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
