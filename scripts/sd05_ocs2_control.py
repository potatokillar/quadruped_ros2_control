#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from control_input_msgs.msg import Inputs
from geometry_msgs.msg import Twist
from rclpy.node import Node


PUBLISH_RATE = 50.0
COMMAND_PULSE_TIME = 0.3
RAMP_TIME = 2.0
INITIAL_RELEASE_TIMEOUT = 0.65
REPEATED_RELEASE_TIMEOUT = 0.18
KEY_REPEAT_CHAIN_TIMEOUT = 0.8

MAX_LINEAR_SPEED = 0.5
MAX_YAW_SPEED = 1.57
MIN_SPEED_RATIO = 0.1

GAITS = {
    "2": (2, "stance"),
    "3": (3, "trot"),
    "4": (9, "static_walk"),
    "5": (8, "dynamic_walk"),
}


class Sd05Ocs2Control(Node):
    def __init__(self):
        super().__init__("sd05_ocs2_control")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.control_input_publisher = self.create_publisher(Inputs, "/control_input", 10)
        self.twist = Twist()
        self.control_input = Inputs()
        self.mode = "unknown"
        self.active_key = None
        self.key_started_at = 0.0
        self.last_key_time = 0.0
        self.release_deadline = 0.0
        self.running = True

    def publish(self):
        self.cmd_vel_publisher.publish(self.twist)
        self.control_input_publisher.publish(self.control_input)

    def pulse_command(self, command):
        self.stop_motion()
        deadline = time.monotonic() + COMMAND_PULSE_TIME
        self.control_input.command = command
        while self.running and rclpy.ok() and time.monotonic() < deadline:
            self.publish()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_RATE)
        self.control_input.command = 0
        self.publish()

    def switch_passive(self):
        print("\nSwitching to passive mode...")
        self.pulse_command(1)
        self.mode = "passive"

    def switch_gait(self, key):
        command, name = GAITS[key]
        print(f"\nSwitching to {name}...")
        self.pulse_command(command)
        self.mode = name

    def stop_motion(self):
        self.twist = Twist()
        self.active_key = None
        self.release_deadline = 0.0
        self.cmd_vel_publisher.publish(self.twist)

    def update_motion(self, key):
        if self.mode not in {name for _, name in GAITS.values()}:
            print("\nPress 2 to enter OCS2 stance, then select a walking gait.")
            return

        now = time.monotonic()
        repeated = key == self.active_key and now - self.last_key_time <= KEY_REPEAT_CHAIN_TIMEOUT
        if not repeated:
            self.key_started_at = now

        self.active_key = key
        self.last_key_time = now
        self.release_deadline = now + (REPEATED_RELEASE_TIMEOUT if repeated else INITIAL_RELEASE_TIMEOUT)

        ratio = min(MIN_SPEED_RATIO + (now - self.key_started_at) / RAMP_TIME, 1.0)
        self.twist = Twist()
        if key == "w":
            self.twist.linear.x = ratio * MAX_LINEAR_SPEED
        elif key == "s":
            self.twist.linear.x = -ratio * MAX_LINEAR_SPEED
        elif key == "a":
            self.twist.linear.y = ratio * MAX_LINEAR_SPEED
        elif key == "d":
            self.twist.linear.y = -ratio * MAX_LINEAR_SPEED
        elif key == "q":
            self.twist.angular.z = ratio * MAX_YAW_SPEED
        elif key == "e":
            self.twist.angular.z = -ratio * MAX_YAW_SPEED

    def stop_if_released(self):
        if self.active_key is not None and time.monotonic() >= self.release_deadline:
            self.stop_motion()

    def handle_key(self, key):
        if key == "1":
            self.switch_passive()
        elif key in GAITS:
            self.switch_gait(key)
        elif key in ("w", "s", "a", "d", "q", "e"):
            self.update_motion(key)
        elif key in (" ", "0"):
            self.stop_motion()
        elif key in ("x", "\x03"):
            self.running = False

    def status(self):
        return (
            f"mode={self.mode:<12} "
            f"vx={self.twist.linear.x:+.2f} "
            f"vy={self.twist.linear.y:+.2f} "
            f"yaw={self.twist.angular.z:+.2f}"
        )

    def run(self):
        print("SD05 OCS2 keyboard control")
        print("  1: passive    2: stance    3: trot")
        print("  4: static walk    5: dynamic walk")
        print("  W/S: forward/back    A/D: left/right    Q/E: turn")
        print("  Space: stop motion    X: exit")
        print("Speed increases while a key is held and resets to zero after release.")

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                if readable:
                    self.handle_key(sys.stdin.read(1).lower())
                self.stop_if_released()
                self.publish()
                rclpy.spin_once(self, timeout_sec=0.0)
                sys.stdout.write("\r" + self.status() + "  ")
                sys.stdout.flush()
                time.sleep(1.0 / PUBLISH_RATE)
        finally:
            self.stop_motion()
            for _ in range(5):
                self.publish()
                time.sleep(1.0 / PUBLISH_RATE)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\nControl stopped; linear and angular velocity are zero.")


def main():
    rclpy.init()
    node = Sd05Ocs2Control()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
