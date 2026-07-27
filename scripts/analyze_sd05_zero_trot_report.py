#!/usr/bin/env python3
"""Analyze SD05 zero-velocity trot experiment rosbag and produce a report."""

import math
import argparse
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from control_input_msgs.msg import Inputs


def q_to_euler(q):
    """Convert quaternion (x, y, z, w) to roll, pitch, yaw (rad)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def yaw_delta(y0, y1):
    """Signed smallest angle difference in radians."""
    d = y1 - y0
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def load_bag(path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    robot_poses = []          # (t, transform)
    cmd_vels = []             # (t, Twist)
    control_inputs = []       # (t, Inputs)
    contacts = []             # (t, set(contact_feet))

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
        if topic == '/world/empty/pose/info':
            msg = deserialize_message(data, TFMessage)
            for tr in msg.transforms:
                if tr.child_frame_id == 'robot':
                    robot_poses.append((ts, tr.transform))
        elif topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            cmd_vels.append((ts, msg))
        elif topic == '/control_input':
            msg = deserialize_message(data, Inputs)
            control_inputs.append((ts, msg))
        elif topic == '/ground_contacts':
            msg = deserialize_message(data, Contacts)
            feet = set()
            for c in msg.contacts:
                name = c.collision2.name if 'ground_plane' in c.collision1.name else c.collision1.name
                # Match both foot-link and contact-pad collision names.
                for foot in ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot',
                             'FL_contact_pad', 'FR_contact_pad',
                             'RL_contact_pad', 'RR_contact_pad'):
                    if foot in name:
                        feet.add(foot[:2])
                        break
            contacts.append((ts, feet))

    return robot_poses, cmd_vels, control_inputs, contacts


def determine_phases(control_inputs):
    """Return key timestamps for stance baseline, trot, and final stance."""
    by_cmd = defaultdict(list)
    for t, msg in control_inputs:
        by_cmd[msg.command].append(t)

    # command 3 triggers trot; command 2 returns to stance.
    cmd3_start = min(by_cmd[3])
    cmd3_end = max(by_cmd[3])
    # Use the first command-2 burst after the trot command; earlier command-2
    # bursts belong to initialization / stance baseline.
    cmd2_after_trot = [t for t in by_cmd[2] if t > cmd3_end]
    cmd2_start = min(cmd2_after_trot) if cmd2_after_trot else max(by_cmd[2])

    # Trot phase: from end of command-3 burst to start of command-2 burst.
    trot_start = cmd3_end
    trot_end = cmd2_start

    # Stance baseline: last ~8 s before the command-3 burst (still in stance).
    stance_end = cmd3_start
    stance_start = stance_end - 8.002

    return stance_start, stance_end, trot_start, trot_end


def interpolate_pose(poses, t):
    """Linearly interpolate pose at time t from sorted poses."""
    if not poses:
        return None
    if t <= poses[0][0]:
        return poses[0][1]
    if t >= poses[-1][0]:
        return poses[-1][1]
    # find bracket
    for i in range(len(poses) - 1):
        t0, p0 = poses[i]
        t1, p1 = poses[i + 1]
        if t0 <= t <= t1:
            if t1 == t0:
                return p0
            alpha = (t - t0) / (t1 - t0)
            # linear interpolation for translation
            interp = lambda a, b: a + alpha * (b - a)
            # simple slerp-ish interpolation not needed for position; use lerp
            from copy import deepcopy
            tr = deepcopy(p0)
            tr.translation.x = interp(p0.translation.x, p1.translation.x)
            tr.translation.y = interp(p0.translation.y, p1.translation.y)
            tr.translation.z = interp(p0.translation.z, p1.translation.z)
            tr.rotation.x = interp(p0.rotation.x, p1.rotation.x)
            tr.rotation.y = interp(p0.rotation.y, p1.rotation.y)
            tr.rotation.z = interp(p0.rotation.z, p1.rotation.z)
            tr.rotation.w = interp(p0.rotation.w, p1.rotation.w)
            return tr
    return poses[-1][1]


def pose_to_state(transform):
    r, p, y = q_to_euler(transform.rotation)
    return {
        'x': transform.translation.x,
        'y': transform.translation.y,
        'z': transform.translation.z,
        'roll': r,
        'pitch': p,
        'yaw': y,
    }


def window_stats(poses, t0, t1):
    samples = [p for tp, p in poses if t0 <= tp <= t1]
    if not samples:
        return None
    states = [pose_to_state(p) for p in samples]
    x0, y0, yaw0 = states[0]['x'], states[0]['y'], states[0]['yaw']
    x1, y1, yaw1 = states[-1]['x'], states[-1]['y'], states[-1]['yaw']
    dx = x1 - x0
    dy = y1 - y0
    dyaw = yaw_delta(yaw0, yaw1)
    rolls = [s['roll'] for s in states]
    pitches = [s['pitch'] for s in states]
    return {
        'duration': states[-1] if isinstance(states[-1], float) else samples[-1][0] - samples[0][0],
        'start': (x0, y0, yaw0),
        'end': (x1, y1, yaw1),
        'dx': dx,
        'dy': dy,
        ' planar': math.hypot(dx, dy),
        'dyaw_deg': math.degrees(dyaw),
        'roll_min_deg': math.degrees(min(rolls)),
        'roll_max_deg': math.degrees(max(rolls)),
        'pitch_min_deg': math.degrees(min(pitches)),
        'pitch_max_deg': math.degrees(max(pitches)),
    }


def phase_average_velocity(poses, contacts, t0, t1, active_feet):
    """Average world-frame velocity over samples where exactly active_feet are in contact."""
    # Build contact state lookup (nearest in time)
    contact_idx = 0
    vx_sum, vy_sum, yawrate_sum, n = 0.0, 0.0, 0.0, 0
    prev = None
    for t, p in poses:
        if t < t0 or t > t1:
            continue
        # advance contact_idx to just <= t
        while contact_idx + 1 < len(contacts) and contacts[contact_idx + 1][0] <= t:
            contact_idx += 1
        _, feet = contacts[contact_idx]
        if feet == active_feet:
            state = pose_to_state(p)
            if prev is not None:
                dt = t - prev[0]
                if dt > 1e-6:
                    vx_sum += (state['x'] - prev[1]['x']) / dt
                    vy_sum += (state['y'] - prev[1]['y']) / dt
                    yawrate_sum += yaw_delta(prev[1]['yaw'], state['yaw']) / dt
                    n += 1
            prev = (t, state)
    if n == 0:
        return None
    return vx_sum / n, vy_sum / n, yawrate_sum / n, n


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', default='/tmp/sd05_zero_trot_20260722', nargs='?')
    args = parser.parse_args()

    robot_poses, cmd_vels, control_inputs, contacts = load_bag(args.bag)
    stance_start, stance_end, trot_start, trot_end = determine_phases(control_inputs)

    print('=== Phase boundaries (s since Unix epoch) ===')
    print(f'Stance baseline: {stance_start:.3f} .. {stance_end:.3f}')
    print(f'Trot phase:      {trot_start:.3f} .. {trot_end:.3f}')

    # Input verification
    trot_cmd = [m for t, m in cmd_vels if trot_start <= t <= trot_end]
    max_cmd = [max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z)) for m in trot_cmd]
    print(f'\n=== /cmd_vel during trot ===')
    print(f'Count: {len(trot_cmd)}')
    print(f'Max abs(linear.x/linear.y/angular.z): {max(max_cmd) if max_cmd else 0}')

    # Stance baseline
    stance_poses = [(t, p) for t, p in robot_poses if stance_start <= t <= stance_end]
    s0 = pose_to_state(stance_poses[0][1])
    s1 = pose_to_state(stance_poses[-1][1])
    stance_dx = s1['x'] - s0['x']
    stance_dy = s1['y'] - s0['y']
    stance_dyaw = yaw_delta(s0['yaw'], s1['yaw'])
    print(f'\n=== Stance baseline ({stance_end - stance_start:.3f} s) ===')
    print(f'Planar displacement: {math.hypot(stance_dx, stance_dy)*1000:.3f} mm')
    print(f'Yaw change: {math.degrees(stance_dyaw):.3f} deg')

    # Trot drift
    trot_poses = [(t, p) for t, p in robot_poses if trot_start <= t <= trot_end]
    t0s = pose_to_state(trot_poses[0][1])
    t1s = pose_to_state(trot_poses[-1][1])
    dx = t1s['x'] - t0s['x']
    dy = t1s['y'] - t0s['y']
    dyaw = yaw_delta(t0s['yaw'], t1s['yaw'])
    rolls = [pose_to_state(p)['roll'] for _, p in trot_poses]
    pitches = [pose_to_state(p)['pitch'] for _, p in trot_poses]
    print(f'\n=== Zero-velocity trot ({trot_poses[-1][0] - trot_poses[0][0]:.3f} s) ===')
    print(f'Start: x={t0s["x"]:.5f} m, y={t0s["y"]:.5f} m')
    print(f'End:   x={t1s["x"]:.5f} m, y={t1s["y"]:.5f} m')
    print(f'dx={dx:.5f} m, dy={dy:.5f} m')
    print(f'Planar displacement: {math.hypot(dx, dy):.5f} m')
    print(f'Yaw change: {math.degrees(dyaw):.3f} deg')
    print(f'Roll range:  {math.degrees(min(rolls)):.3f} .. {math.degrees(max(rolls)):.3f} deg')
    print(f'Pitch range: {math.degrees(min(pitches)):.3f} .. {math.degrees(max(pitches)):.3f} deg')

    # Diagonal support phase velocities
    print('\n=== Diagonal support phase average velocities ===')
    for label, feet in [('FL+RR', {'FL', 'RR'}), ('FR+RL', {'FR', 'RL'})]:
        res = phase_average_velocity(robot_poses, contacts, trot_start, trot_end, feet)
        if res:
            vx, vy, vyaw, n = res
            print(f'{label}: vx={vx:.5f} m/s, vy={vy:.5f} m/s, yaw_rate={vyaw:.5f} rad/s (n={n})')
        else:
            print(f'{label}: no samples')


if __name__ == '__main__':
    main()
