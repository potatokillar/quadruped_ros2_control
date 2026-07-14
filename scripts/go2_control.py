#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from control_input_msgs.msg import Inputs
from rclpy.node import Node
from std_srvs.srv import Empty


PUBLISH_RATE = 50.0
STATE_TRANSITION_TIME = 2.1
COMMAND_PULSE_TIME = 0.25
RAMP_TIME = 2.0
INITIAL_RELEASE_TIMEOUT = 0.75
REPEATED_RELEASE_TIMEOUT = 0.20
KEY_REPEAT_CHAIN_TIMEOUT = 0.8


class Go2Control(Node):
    def __init__(self):
        super().__init__("go2_control")
        self.publisher = self.create_publisher(Inputs, "control_input", 10)
        self.reset_world_client = self.create_client(Empty, "/reset_world")
        self.message = Inputs()
        self.mode = "unknown"
        self.last_motion_key = None
        self.last_motion_key_time = 0.0
        self.motion_key_start_time = 0.0
        self.motion_release_deadline = 0.0
        self.motion_key_repeated = False
        self.running = True

    def publish(self, command=None):
        if command is not None:
            self.message.command = command
        self.publisher.publish(self.message)

    def publish_for(self, command, duration):
        deadline = time.monotonic() + duration
        self.message.command = command
        while self.running and rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(self.message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_RATE)
        self.message.command = 0
        self.publisher.publish(self.message)

    def wait_transition(self, duration):
        deadline = time.monotonic() + duration
        while self.running and rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(self.message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_RATE)

    def clear_motion(self):
        self.message.lx = 0.0
        self.message.ly = 0.0
        self.message.rx = 0.0
        self.message.ry = 0.0
        self.last_motion_key = None
        self.motion_release_deadline = 0.0
        self.motion_key_repeated = False
        self.publish(0)

    def passive(self):
        self.clear_motion()
        print("\nSwitching to passive mode...")
        self.publish_for(1, STATE_TRANSITION_TIME)
        self.mode = "passive"

    def down(self):
        self.clear_motion()
        print("\nMoving to fixed-down pose...")
        self.publish_for(1, STATE_TRANSITION_TIME)
        self.publish_for(2, COMMAND_PULSE_TIME)
        self.wait_transition(STATE_TRANSITION_TIME)
        self.mode = "down"

    def stand(self):
        self.clear_motion()
        print("\nStanding up...")
        # Force a known state, then pass through FIXEDDOWN to FIXEDSTAND.
        self.publish_for(1, STATE_TRANSITION_TIME)
        self.publish_for(2, STATE_TRANSITION_TIME)
        self.wait_transition(STATE_TRANSITION_TIME)
        self.mode = "stand"

    def trot(self):
        if self.mode != "stand":
            self.stand()
        print("\nSwitching to trotting mode...")
        self.publish_for(4, COMMAND_PULSE_TIME)
        self.mode = "trot"

    def recover_simulation(self):
        self.clear_motion()
        print("\nResetting Gazebo world and restoring the standing pose...")
        self.publish_for(1, STATE_TRANSITION_TIME)
        if not self.reset_world_client.wait_for_service(timeout_sec=2.0):
            print("\n/reset_world is unavailable; recovery requires Gazebo Classic.")
            return
        future = self.reset_world_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.exception() is not None:
            print("\nGazebo reset failed.")
            return
        self.mode = "passive"
        time.sleep(0.5)
        self.stand()

    def ramped_value(self, key):
        now = time.monotonic()
        repeated = (
            key == self.last_motion_key
            and now - self.last_motion_key_time <= KEY_REPEAT_CHAIN_TIMEOUT
        )
        if not repeated:
            self.motion_key_start_time = now
        self.motion_key_repeated = repeated
        self.last_motion_key = key
        self.last_motion_key_time = now
        timeout = REPEATED_RELEASE_TIMEOUT if repeated else INITIAL_RELEASE_TIMEOUT
        self.motion_release_deadline = now + timeout
        return min((now - self.motion_key_start_time) / RAMP_TIME + 0.1, 1.0)

    def handle_motion(self, key):
        if self.mode != "trot":
            print("\nPress 4 to enter trotting mode before moving.")
            return

        value = self.ramped_value(key)
        self.message.lx = 0.0
        self.message.ly = 0.0
        self.message.rx = 0.0
        self.message.ry = 0.0

        if key == "w":
            self.message.ly = value
        elif key == "s":
            self.message.ly = -value
        elif key == "a":
            self.message.lx = -value
        elif key == "d":
            self.message.lx = value
        elif key in ("j", "q"):
            self.message.rx = -value
        elif key in ("l", "e"):
            self.message.rx = value
        self.publish(0)

    def stop_if_released(self):
        if (
            self.last_motion_key is not None
            and time.monotonic() >= self.motion_release_deadline
        ):
            self.clear_motion()

    def handle_key(self, key):
        if key == "1":
            self.passive()
        elif key == "2":
            self.down()
        elif key == "3":
            self.stand()
        elif key == "4":
            self.trot()
        elif key == "r":
            self.recover_simulation()
        elif key in ("w", "s", "a", "d", "j", "l", "q", "e"):
            self.handle_motion(key)
        elif key in (" ", "0"):
            self.clear_motion()
        elif key in ("x", "\x03"):
            self.running = False

    def status(self):
        return (
            "mode={:<7} forward={:+.2f} lateral={:+.2f} yaw={:+.2f}".format(
                self.mode, self.message.ly, -self.message.lx, -self.message.rx
            )
        )

    def run(self):
        print("Go2 keyboard control")
        print("  1: passive    2: down    3: stand    4: trot")
        print("  W/S: forward/back    A/D: left/right    J/L or Q/E: turn")
        print("  R: Gazebo recovery    Space: stop motion    X: exit")
        print("Movement speed increases while a key is held and resets after release.")

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                if readable:
                    self.handle_key(sys.stdin.read(1).lower())
                self.stop_if_released()
                self.publisher.publish(self.message)
                rclpy.spin_once(self, timeout_sec=0.0)
                sys.stdout.write("\r" + self.status() + "  ")
                sys.stdout.flush()
                time.sleep(1.0 / PUBLISH_RATE)
        finally:
            self.clear_motion()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\nControl stopped; motion command has been reset to zero.")


def main():
    rclpy.init()
    node = Go2Control()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
