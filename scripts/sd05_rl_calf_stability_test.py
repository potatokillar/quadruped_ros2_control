#!/usr/bin/env python3

import csv
import os
import time
from pathlib import Path

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


JOINTS = {
    "RL_hip_joint": "/rl_hip_diagnostic_controller/commands",
    "RL_thigh_joint": "/rl_thigh_diagnostic_controller/commands",
    "RL_calf_joint": "/rl_calf_diagnostic_controller/commands",
}
INTERFACES = ("position", "velocity", "effort", "kp", "kd")
CALF_CENTER = -2.0
TEST_VELOCITIES = (-0.2, -2.2, -6.4)
KD_VALUES = (0.0, 2.0, 3.5, 5.5)


class StabilityTest(Node):
    def __init__(self):
        super().__init__("sd05_rl_calf_stability_test")
        self.command_publishers = {
            joint: self.create_publisher(Float64MultiArray, topic, 10)
            for joint, topic in JOINTS.items()
        }
        self.subscription = self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_state, qos_profile_sensor_data)
        self.states = {}
        self.latest_stamp = 0.0
        self.active_label = ""
        self.active_kd = 0.0
        self.active_velocity = 0.0
        self.samples = []

    def on_state(self, message):
        self.latest_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        for name, values in zip(message.joint_names, message.interface_values):
            if name in JOINTS:
                self.states[name] = dict(zip(values.interface_names, values.values))
        if self.active_label and "RL_calf_joint" in self.states:
            state = self.states["RL_calf_joint"]
            self.samples.append({
                "test": self.active_label,
                "kd": self.active_kd,
                "target_velocity": self.active_velocity,
                "sim_time": self.latest_stamp,
                "position": state.get("position", float("nan")),
                "velocity": state.get("velocity", float("nan")),
                "measured_effort": state.get("effort", float("nan")),
                "feedforward_effort": state.get("feedforward_effort", float("nan")),
                "position_feedback_effort": state.get("position_feedback_effort", float("nan")),
                "velocity_feedback_effort": state.get("velocity_feedback_effort", float("nan")),
                "commanded_effort": state.get("commanded_effort", float("nan")),
            })

    def publish(self, joint, position, velocity, effort, kp, kd):
        message = Float64MultiArray()
        message.data = [position, velocity, effort, kp, kd]
        self.command_publishers[joint].publish(message)

    def run_for(self, duration, calf_command, label="", kd=0.0, target_velocity=0.0):
        self.active_label = label
        self.active_kd = kd
        self.active_velocity = target_velocity
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish("RL_hip_joint", self.hip_position, 0.0, 0.0, 20.0, 1.0)
            self.publish("RL_thigh_joint", self.thigh_position, 0.0, 0.0, 20.0, 1.0)
            self.publish("RL_calf_joint", *calf_command())
            rclpy.spin_once(self, timeout_sec=0.001)
        self.active_label = ""

    def wait_for_state(self):
        deadline = time.monotonic() + 10.0
        while len(self.states) < len(JOINTS) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if len(self.states) < len(JOINTS):
            raise RuntimeError("Timed out waiting for RL joint states")
        self.hip_position = self.states["RL_hip_joint"]["position"]
        self.thigh_position = self.states["RL_thigh_joint"]["position"]

    def move_calf_to_center(self):
        start_position = self.states["RL_calf_joint"]["position"]
        start = time.monotonic()

        def command():
            ratio = min((time.monotonic() - start) / 1.5, 1.0)
            target = start_position + ratio * (CALF_CENTER - start_position)
            return target, 0.0, 0.0, 5.0, 1.0

        self.run_for(2.0, command)
        self.run_for(0.5, lambda: (CALF_CENTER, 0.0, 0.0, 5.0, 1.0))

    def run_tests(self):
        self.wait_for_state()
        for velocity in TEST_VELOCITIES:
            for kd in KD_VALUES:
                self.move_calf_to_center()
                label = f"v_{velocity:.1f}_kd_{kd:.1f}"
                self.run_for(
                    0.08,
                    lambda kd=kd, velocity=velocity: (CALF_CENTER, velocity, 0.0, 0.0, kd),
                    label=label,
                    kd=kd,
                    target_velocity=velocity,
                )
                self.run_for(0.3, lambda: (CALF_CENTER, 0.0, 0.0, 5.0, 1.0))


def summarize(samples):
    print("target   kd count max|dq| max|tau_cmd| delta_dq_flips saturation")
    for velocity in TEST_VELOCITIES:
        for kd in KD_VALUES:
            label = f"v_{velocity:.1f}_kd_{kd:.1f}"
            rows = [row for row in samples if row["test"] == label]
            increments = [b["velocity"] - a["velocity"] for a, b in zip(rows, rows[1:])]
            flips = sum(a * b < 0.0 for a, b in zip(increments, increments[1:]))
            saturation = sum(abs(row["measured_effort"]) >= 34.9 for row in rows)
            fraction = saturation / len(rows) if rows else 0.0
            print(f"{velocity:+6.1f} {kd:4.1f} {len(rows):5d} "
                  f"{max(map(abs, (r['velocity'] for r in rows)), default=0):7.3f} "
                  f"{max(map(abs, (r['commanded_effort'] for r in rows)), default=0):12.3f} "
                  f"{flips:14d} {fraction:9.1%}")


def main():
    rclpy.init()
    node = StabilityTest()
    try:
        node.run_tests()
        data_root = Path(os.environ.get(
            "SD05_DATA_ROOT", "/media/wl/data/quadruped_ros2_control"))
        output = data_root / "experiments" / "sd05_rl_calf_stability.csv"
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=node.samples[0].keys())
            writer.writeheader()
            writer.writerows(node.samples)
        summarize(node.samples)
        print(f"samples: {output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
