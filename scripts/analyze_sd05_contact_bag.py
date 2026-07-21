#!/usr/bin/env python3

import argparse
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


LEGS = ("FL", "FR", "RL", "RR")


def quaternion_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def mode_contacts(mode):
    return frozenset(leg for index, leg in enumerate(LEGS) if mode & (1 << index))


def format_contacts(contacts):
    return "+".join(leg for leg in LEGS if leg in contacts) or "none"


def nearest(samples, timestamp):
    if not samples:
        return None
    lo, hi = 0, len(samples)
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] < timestamp:
            lo = mid + 1
        else:
            hi = mid
    candidates = samples[max(0, lo - 1):min(len(samples), lo + 1)]
    return min(candidates, key=lambda item: abs(item[0] - timestamp))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class BagReader:
    def __init__(self, bag_path):
        bag_path = Path(bag_path)
        database = bag_path if bag_path.suffix == ".db3" else next(bag_path.glob("*.db3"))
        self.connection = sqlite3.connect(database)
        rows = self.connection.execute("SELECT id, name, type FROM topics")
        self.topics = {name: (topic_id, get_message(type_name)) for topic_id, name, type_name in rows}

    def read(self, topic, start=None, end=None):
        topic_id, message_type = self.topics[topic]
        query = "SELECT timestamp, data FROM messages WHERE topic_id = ?"
        values = [topic_id]
        if start is not None:
            query += " AND timestamp >= ?"
            values.append(start)
        if end is not None:
            query += " AND timestamp <= ?"
            values.append(end)
        query += " ORDER BY timestamp"
        return [
            (timestamp, deserialize_message(data, message_type))
            for timestamp, data in self.connection.execute(query, values)
        ]


def pose_sample(message):
    for transform in message.transforms:
        if transform.child_frame_id == "robot":
            roll, pitch, yaw = quaternion_to_rpy(transform.transform.rotation)
            return roll, pitch, yaw, transform.transform.translation.z
    return None


def contact_sample(message):
    feet = set()
    other = set()
    for contact in message.contacts:
        names = (contact.collision1.name, contact.collision2.name)
        robot_collision = next((name for name in names if name.startswith("robot::")), None)
        if robot_collision is None:
            continue
        leg = next((leg for leg in LEGS if f"::{leg}_foot::{leg}_foot_collision" in robot_collision), None)
        if leg:
            feet.add(leg)
        else:
            other.add(robot_collision)
    return frozenset(feet), frozenset(other)


def dynamic_sample(message):
    force_index = message.joint_names.index("foot_force")
    force_values = message.interface_values[force_index]
    forces = dict(zip(force_values.interface_names, force_values.values))
    feet = frozenset(leg for leg in LEGS if forces.get(f"{leg}_foot_force", 0.0) > 4.0)
    joints = {}
    for name, values in zip(message.joint_names, message.interface_values):
        if name in ("foot_force", "imu_sensor"):
            continue
        joints[name] = dict(zip(values.interface_names, values.values))
    efforts = [joint.get("effort", 0.0) for joint in joints.values()]
    return forces, feet, max(map(abs, efforts), default=0.0), joints


def command_sample(message):
    joints = {
        name: dict(zip(values.interface_names, values.values))
        for name, values in zip(message.joint_names, message.interface_values)
    }
    efforts = [joint.get("effort", 0.0) for joint in joints.values()]
    return max(map(abs, efforts), default=0.0), joints


def diagnostic_sample(message):
    if not message.status:
        return {}
    return {item.key: float(item.value) for item in message.status[0].values}


def first_event(samples, predicate):
    return next(((timestamp, value) for timestamp, value in samples if predicate(value)), None)


def main():
    parser = argparse.ArgumentParser(description="Analyze SD05 contact truth around a gait switch")
    parser.add_argument("bag")
    parser.add_argument("--before", type=float, default=1.0)
    parser.add_argument("--after", type=float, default=5.0)
    args = parser.parse_args()

    bag = BagReader(args.bag)
    controls = bag.read("/control_input")
    gait_switch = next((timestamp for timestamp, message in controls if message.command == 3), None)
    if gait_switch is None:
        raise RuntimeError("No command=3 message found")
    start = gait_switch - int(args.before * 1e9)
    end = gait_switch + int(args.after * 1e9)

    pose_messages = bag.read("/world/empty/pose/info", start, end)
    contacts = [(stamp_ns(msg.header.stamp), contact_sample(msg))
                for _, msg in bag.read("/ground_contacts", start, end)]
    dynamics = [(stamp_ns(msg.header.stamp), dynamic_sample(msg))
                for _, msg in bag.read("/dynamic_joint_states", start, end)]
    measured = [(round(msg.time * 1e9), (msg.mode, msg.state.value[8])) for _, msg in bag.read(
        "/ocs2_quadruped_controller/measured_observation", start, end)]
    desired_messages = bag.read("/ocs2_quadruped_controller/desired_observation", start, end)
    desired = [(round(msg.time * 1e9), (msg.mode, msg.state.value[8]))
               for _, msg in desired_messages]
    commands = [(stamp_ns(msg.header.stamp), command_sample(msg)) for _, msg in bag.read(
        "/ocs2_quadruped_controller/joint_commands", start, end)]
    diagnostics = [(stamp_ns(msg.header.stamp), diagnostic_sample(msg)) for _, msg in bag.read(
        "/ocs2_quadruped_controller/wbc_diagnostics", start, end)]

    sim_switch = next((timestamp for timestamp, value in desired if value[0] != 15), None)
    if sim_switch is None:
        raise RuntimeError("Desired mode did not leave stance in the analysis window")
    desired_bag_switch = next(
        timestamp for timestamp, msg in desired_messages if msg.mode != 15)
    bag_to_sim_offset = sim_switch - desired_bag_switch
    poses = [(timestamp + bag_to_sim_offset, value)
             for timestamp, msg in pose_messages if (value := pose_sample(msg)) is not None]

    def relative(timestamp):
        return (timestamp - sim_switch) / 1e9

    events = []
    event = first_event(desired, lambda value: value[0] != 15)
    if event:
        mode = event[1][0]
        events.append((event[0], f"desired mode {mode} ({format_contacts(mode_contacts(mode))})"))
    event = first_event(measured, lambda value: value[0] != 15)
    if event:
        mode = event[1][0]
        events.append((event[0], f"measured mode {mode} ({format_contacts(mode_contacts(mode))})"))
    event = first_event(contacts, lambda value: value[0] != frozenset(LEGS))
    if event:
        events.append((event[0], f"ground feet {format_contacts(event[1][0])}; other={len(event[1][1])}"))
    event = first_event(contacts, lambda value: bool(value[1]))
    if event:
        events.append((event[0], "non-foot ground contact: " + ", ".join(sorted(event[1][1]))))
    event = first_event(poses, lambda value: max(abs(value[0]), abs(value[1])) > math.radians(5.0))
    if event:
        events.append((event[0], f"true attitude >5deg: roll={math.degrees(event[1][0]):.2f}, "
                                  f"pitch={math.degrees(event[1][1]):.2f}"))
    event = first_event(poses, lambda value: max(abs(value[0]), abs(value[1])) > math.radians(1.0))
    if event:
        events.append((event[0], f"true attitude >1deg: roll={math.degrees(event[1][0]):.2f}, "
                                  f"pitch={math.degrees(event[1][1]):.2f}"))
    event = first_event(commands, lambda value: value[0] >= 34.9)
    if event:
        events.append((event[0], f"command torque saturation: {event[1][0]:.3f} Nm"))
    event = first_event(dynamics, lambda value: value[2] >= 34.9)
    if event:
        timestamp, dynamic = event
        joint_name, actual = max(
            dynamic[3].items(), key=lambda item: abs(item[1].get("effort", 0.0)))
        command = nearest(commands, timestamp)[1][1][joint_name]
        if "commanded_effort" in actual:
            feedforward = actual["feedforward_effort"]
            p_term = actual["position_feedback_effort"]
            d_term = actual["velocity_feedback_effort"]
            total = actual["commanded_effort"]
        else:
            feedforward = command.get("effort", 0.0)
            p_term = command.get("kp", 0.0) * (
                command.get("position", 0.0) - actual.get("position", 0.0))
            d_term = command.get("kd", 0.0) * (
                command.get("velocity", 0.0) - actual.get("velocity", 0.0))
            total = feedforward + p_term + d_term
        events.append((timestamp, f"actual joint torque saturation: {joint_name}="
                                  f"{actual.get('effort', 0.0):.3f} Nm; ff={feedforward:.2f}, "
                                  f"P={p_term:.2f}, D={d_term:.2f}, total={total:.2f}; "
                                  f"q={actual.get('position', 0.0):.3f}->{command.get('position', 0.0):.3f}, "
                                  f"dq={actual.get('velocity', 0.0):.3f}->{command.get('velocity', 0.0):.3f}, "
                                  f"kp={command.get('kp', 0.0):.1f}, kd={command.get('kd', 0.0):.1f}"))

    for threshold in (0.01, 0.05):
        height_errors = []
        for timestamp, pose in poses:
            measured_value = nearest(measured, timestamp)
            if measured_value:
                height_errors.append((timestamp, (measured_value[1][1] - pose[3], pose[3], measured_value[1][1])))
        event = first_event(height_errors, lambda value: abs(value[0]) > threshold)
        if event:
            events.append((event[0], f"Kalman height error >{threshold * 100:.0f}cm: "
                                      f"true={event[1][1]:.3f}m, estimated={event[1][2]:.3f}m"))

    mismatches = []
    for timestamp, dynamic in dynamics:
        contact = nearest(contacts, timestamp)
        if contact and dynamic[1] != contact[1][0]:
            mismatches.append((timestamp, (dynamic[1], contact[1][0])))
    if mismatches:
        timestamp, value = mismatches[0]
        dynamic = nearest(dynamics, timestamp)[1]
        force_text = ", ".join(
            f"{leg}={dynamic[0].get(f'{leg}_foot_force', float('nan')):.2f}N" for leg in LEGS)
        events.append((timestamp, f"force>4 vs ground mismatch: {format_contacts(value[0])} vs "
                                  f"{format_contacts(value[1])}; {force_text}"))

    print(f"command=3 bag timestamp: {gait_switch}")
    print(f"desired gait switch simulation timestamp: {sim_switch / 1e9:.6f}s")
    print("\nFirst events (ordered):")
    for timestamp, description in sorted(events):
        print(f"  {relative(timestamp):+8.4f}s  {description}")

    print("\nTimeline:")
    print(" t(s)   roll   pitch  trueZ  estZ  des meas  ground       force>4      cmdTau actTau wbcTau")
    for offset in (-0.5, -0.1, 0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30,
                   0.40, 0.50, 0.75, 1.00, 1.50, 2.00, 3.00, 4.00):
        timestamp = sim_switch + int(offset * 1e9)
        pose = nearest(poses, timestamp)
        contact = nearest(contacts, timestamp)
        dynamic = nearest(dynamics, timestamp)
        measured_mode = nearest(measured, timestamp)
        desired_mode = nearest(desired, timestamp)
        command = nearest(commands, timestamp)
        diagnostic = nearest(diagnostics, timestamp)
        if not all((pose, contact, dynamic, measured_mode, desired_mode, command, diagnostic)):
            continue
        roll, pitch, _, height = pose[1]
        wbc_torque = diagnostic[1].get("max_torque", float("nan"))
        desired_mode, _ = desired_mode[1]
        measured_mode, estimated_height = measured_mode[1]
        print(f"{offset:+5.2f} {math.degrees(roll):+7.2f} {math.degrees(pitch):+7.2f} "
              f"{height:6.3f} {estimated_height:5.3f} {desired_mode:4d} {measured_mode:4d}  "
              f"{format_contacts(contact[1][0]):<12} {format_contacts(dynamic[1][1]):<12} "
              f"{command[1][0]:6.2f} {dynamic[1][2]:6.2f} {wbc_torque:6.2f}")


if __name__ == "__main__":
    main()
