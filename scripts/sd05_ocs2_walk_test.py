#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


PUBLISH_RATE = 50.0
MAX_TILT = math.radians(30.0)
MAX_HEIGHT_DROP = 0.15
ODOM_TIMEOUT = 1.0


def quaternion_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class SafetyError(RuntimeError):
    pass


class Sd05WalkTest(Node):
    def __init__(self, args):
        super().__init__("sd05_ocs2_walk_test")
        self.args = args
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_input_publisher = self.create_publisher(Inputs, "/control_input", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 20)
        self.control_input = Inputs()
        self.odom = None
        self.odom_received_at = 0.0
        self.reference_height = None
        self.last_status_at = 0.0

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_received_at = time.monotonic()

    def publish(self, twist=None, command=0):
        self.control_input.command = command
        self.control_input.lx = 0.0
        self.control_input.ly = 0.0
        self.control_input.rx = 0.0
        self.control_input.ry = 0.0
        self.cmd_vel_publisher.publish(twist if twist is not None else Twist())
        self.control_input_publisher.publish(self.control_input)

    def spin_and_sleep(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_RATE)

    def wait_for_odom(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.odom is None and time.monotonic() < deadline:
            self.spin_and_sleep()
        if self.odom is None:
            raise RuntimeError("Timed out waiting for /odom")

    def current_state(self):
        if self.odom is None or time.monotonic() - self.odom_received_at > ODOM_TIMEOUT:
            raise SafetyError("Odometry is stale")
        pose = self.odom.pose.pose
        roll, pitch, yaw = quaternion_to_rpy(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw

    def check_safety(self):
        _, _, height, roll, pitch, _ = self.current_state()
        if abs(roll) > MAX_TILT or abs(pitch) > MAX_TILT:
            raise SafetyError(
                f"Tilt limit exceeded: roll={math.degrees(roll):.1f} deg, "
                f"pitch={math.degrees(pitch):.1f} deg"
            )
        if self.reference_height is not None and height < self.reference_height - MAX_HEIGHT_DROP:
            raise SafetyError(
                f"Height drop exceeded: current={height:.3f} m, "
                f"reference={self.reference_height:.3f} m"
            )

    def print_status(self, phase, extra=""):
        now = time.monotonic()
        if now - self.last_status_at < 1.0:
            return
        x, y, height, roll, pitch, yaw = self.current_state()
        suffix = f" {extra}" if extra else ""
        print(
            f"[{phase}] x={x:+.3f} y={y:+.3f} z={height:.3f} "
            f"roll={math.degrees(roll):+.1f} pitch={math.degrees(pitch):+.1f} "
            f"yaw={math.degrees(yaw):+.1f}{suffix}",
            flush=True,
        )
        self.last_status_at = now

    def publish_for(self, duration, twist, phase, command=0):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish(twist, command)
            self.spin_and_sleep()
            self.check_safety()
            remaining = max(0.0, deadline - time.monotonic())
            self.print_status(phase, f"remaining={remaining:.1f}s")

    def initialize_ocs2(self):
        started_at = time.monotonic()
        settle_deadline = started_at + self.args.settle_time
        initialization_deadline = started_at + 30.0
        self.reference_height = self.current_state()[2]

        while rclpy.ok():
            self.publish(Twist(), command=2)
            self.spin_and_sleep()
            now = time.monotonic()
            odom_is_fresh = now - self.odom_received_at <= ODOM_TIMEOUT

            if odom_is_fresh:
                self.check_safety()
                self.print_status(
                    "initialize",
                    f"remaining={max(0.0, settle_deadline - now):.1f}s",
                )
                if now >= settle_deadline:
                    self.reference_height = self.current_state()[2]
                    return
            elif now >= initialization_deadline:
                raise SafetyError("OCS2 initialization did not resume odometry within 30 s")

    def rotate(self):
        target = math.radians(self.args.turn_degrees)
        direction = 1.0 if target >= 0.0 else -1.0
        twist = Twist()
        twist.angular.z = direction * abs(self.args.yaw_speed)

        _, _, _, _, _, previous_yaw = self.current_state()
        accumulated = 0.0
        nominal_time = abs(target) / abs(self.args.yaw_speed)
        deadline = time.monotonic() + nominal_time * 2.0 + 20.0

        while rclpy.ok() and direction * accumulated < abs(target):
            if time.monotonic() >= deadline:
                raise SafetyError(
                    f"Rotation timed out at {math.degrees(accumulated):.1f} deg"
                )
            self.publish(twist)
            self.spin_and_sleep()
            self.check_safety()
            _, _, _, _, _, yaw = self.current_state()
            accumulated += math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
            previous_yaw = yaw
            self.print_status(
                "rotate",
                f"turned={math.degrees(accumulated):+.1f}/{self.args.turn_degrees:+.1f}deg",
            )

        return accumulated

    def move(self, phase, linear_x, linear_y):
        """按机身坐标系速度执行单个平移测试，并返回世界坐标系位移。"""
        start_x, start_y, _, _, _, _ = self.current_state()
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        print(
            f"Moving {phase}: vx={linear_x:+.2f} m/s, vy={linear_y:+.2f} m/s "
            f"for {self.args.move_duration:.1f} s.",
            flush=True,
        )
        self.publish_for(self.args.move_duration, twist, phase)
        self.publish_for(self.args.stop_time, Twist(), f"{phase}_stop")
        end_x, end_y, _, _, _, _ = self.current_state()
        dx = end_x - start_x
        dy = end_y - start_y
        print(
            f"{phase} completed: dx={dx:+.3f} m, dy={dy:+.3f} m, "
            f"displacement={math.hypot(dx, dy):.3f} m.",
            flush=True,
        )
        return dx, dy

    def switch_to_stance(self):
        print("Switching OCS2 gait to stance and holding zero velocity.", flush=True)
        self.publish_for(1.0, Twist(), "stance", command=2)
        self.publish_for(0.5, Twist(), "stance")

    def switch_to_trot(self):
        print("Switching OCS2 gait from stance to trot.", flush=True)
        self.publish_for(0.5, Twist(), "trot", command=3)
        self.publish_for(0.5, Twist(), "trot")

    def switch_to_passive(self):
        for _ in range(10):
            self.publish(Twist(), command=1)
            self.spin_and_sleep()

    def run(self):
        self.wait_for_odom()
        print("Standing up and entering OCS2 stance.", flush=True)
        self.initialize_ocs2()
        self.switch_to_trot()

        displacement_results = {
            "forward": self.move("forward", self.args.linear_speed, 0.0),
            "backward": self.move("backward", -self.args.linear_speed, 0.0),
            "left": self.move("left", 0.0, self.args.linear_speed),
            "right": self.move("right", 0.0, -self.args.linear_speed),
        }

        print(
            f"Rotating {self.args.turn_degrees:.1f} deg at "
            f"{self.args.yaw_speed:.2f} rad/s with odometry feedback.",
            flush=True,
        )
        turned = self.rotate()
        self.switch_to_stance()

        displacement_text = ", ".join(
            f"{name}={math.hypot(dx, dy):.3f} m"
            for name, (dx, dy) in displacement_results.items()
        )
        print(
            f"Test completed: {displacement_text}, "
            f"accumulated rotation={math.degrees(turned):.1f} deg.",
            flush=True,
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the SD05 OCS2 multidirectional walking and rotation simulation test."
    )
    parser.add_argument("--move-duration", type=float, default=4.0)
    parser.add_argument("--linear-speed", type=float, default=0.10)
    parser.add_argument("--turn-degrees", type=float, default=720.0)
    parser.add_argument("--yaw-speed", type=float, default=0.35)
    parser.add_argument("--settle-time", type=float, default=5.0)
    parser.add_argument("--stop-time", type=float, default=2.0)
    args = parser.parse_args()
    if args.move_duration <= 0.0 or args.settle_time <= 0.0 or args.stop_time < 0.0:
        parser.error("durations must be positive (stop-time may be zero)")
    if args.linear_speed <= 0.0:
        parser.error("linear-speed must be positive")
    if args.yaw_speed <= 0.0:
        parser.error("yaw-speed must be positive")
    if args.turn_degrees == 0.0:
        parser.error("turn-degrees must be non-zero")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = Sd05WalkTest(args)
    exit_code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        print("Test interrupted; switching to OCS2 stance.", flush=True)
        node.switch_to_stance()
        exit_code = 130
    except SafetyError as exc:
        print(f"Safety abort: {exc}", flush=True)
        node.switch_to_passive()
        exit_code = 2
    except Exception as exc:
        print(f"Test failed: {exc}", flush=True)
        node.switch_to_passive()
        exit_code = 1
    finally:
        for _ in range(10):
            node.publish(Twist())
            node.spin_and_sleep()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
