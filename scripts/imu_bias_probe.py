#!/usr/bin/env python3
"""Measure the realized IMU linear-acceleration bias of the current Gazebo run.

Collects IMU samples while the robot is static, rotates measured proper
acceleration into the world frame using the IMU orientation, subtracts
gravity, and reports the residual (the realized constant bias).
"""

import math
import time

import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node


def quat_rotate(x, y, z, w, vx, vy, vz):
    # v' = q * v * q^-1
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class ImuBiasProbe(Node):
    def __init__(self, duration):
        super().__init__("imu_bias_probe")
        self.deadline = time.monotonic() + duration
        self.samples = []
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self.cb, 50)

    def cb(self, msg):
        if time.monotonic() > self.deadline:
            return
        q = msg.orientation
        a = msg.linear_acceleration
        wx, wy, wz = quat_rotate(q.x, q.y, q.z, q.w, a.x, a.y, a.z)
        self.samples.append((wx, wy, wz - 9.81))


def main():
    rclpy.init()
    node = ImuBiasProbe(duration=6.0)
    while rclpy.ok() and time.monotonic() < node.deadline + 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = node.samples
    node.destroy_node()
    rclpy.shutdown()
    n = len(s)
    if n < 100:
        print(f"insufficient samples: {n}")
        raise SystemExit(1)
    mx = sum(v[0] for v in s) / n
    my = sum(v[1] for v in s) / n
    mz = sum(v[2] for v in s) / n
    mag = math.sqrt(mx * mx + my * my + mz * mz)
    print(f"samples={n}")
    print(f"bias_world=({mx:+.4f}, {my:+.4f}, {mz:+.4f}) |b|={mag:.4f}")


if __name__ == "__main__":
    main()
