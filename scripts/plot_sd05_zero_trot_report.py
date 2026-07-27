#!/usr/bin/env python3
"""Generate plots for the SD05 zero-velocity trot experiment report."""

import math
import argparse
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from control_input_msgs.msg import Inputs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def q_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_delta(y0, y1):
    d = y1 - y0
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def load_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    robot_poses = []
    cmd_vels = []
    control_inputs = []
    contacts = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
        if topic == '/world/empty/pose/info':
            msg = deserialize_message(data, TFMessage)
            for tr in msg.transforms:
                if tr.child_frame_id == 'robot':
                    r, p, y = q_to_euler(tr.transform.rotation)
                    robot_poses.append((ts, tr.transform.translation.x,
                                        tr.transform.translation.y,
                                        tr.transform.translation.z, r, p, y))
        elif topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            cmd_vels.append((ts, msg.linear.x, msg.linear.y, msg.angular.z))
        elif topic == '/control_input':
            msg = deserialize_message(data, Inputs)
            control_inputs.append((ts, msg.command))
        elif topic == '/ground_contacts':
            msg = deserialize_message(data, Contacts)
            feet = set()
            for c in msg.contacts:
                name = c.collision2.name if 'ground_plane' in c.collision1.name else c.collision1.name
                for foot in ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'):
                    if foot in name:
                        feet.add(foot[:2])
                        break
            contacts.append((ts, feet))

    return robot_poses, cmd_vels, control_inputs, contacts


def determine_phases(control_inputs):
    by_cmd = defaultdict(list)
    for t, c in control_inputs:
        by_cmd[c].append(t)
    cmd3_start = min(by_cmd[3])
    cmd3_end = max(by_cmd[3])
    cmd2_start = min(by_cmd[2])
    return cmd3_start - 8.002, cmd3_start, cmd3_end, cmd2_start


def contact_state(feet):
    if feet == {'FL', 'RR'}:
        return 1
    if feet == {'FR', 'RL'}:
        return -1
    if len(feet) >= 3:
        return 2
    if len(feet) == 0:
        return 0
    return 3  # other single-leg or transient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', default='/tmp/sd05_zero_trot_20260722', nargs='?')
    parser.add_argument('--output-dir', default='.images')
    args = parser.parse_args()

    robot_poses, cmd_vels, control_inputs, contacts = load_bag(args.bag)
    stance_start, stance_end, trot_start, trot_end = determine_phases(control_inputs)
    t0 = robot_poses[0][0]
    times = [t - t0 for t, *_ in robot_poses]
    xs = [p[1] for p in robot_poses]
    ys = [p[2] for p in robot_poses]
    zs = [p[3] for p in robot_poses]
    rolls = [math.degrees(p[4]) for p in robot_poses]
    pitches = [math.degrees(p[5]) for p in robot_poses]
    yaws = [math.degrees(p[6]) for p in robot_poses]

    # Contact state vs time (interpolated to pose times)
    cidx = 0
    cstates = []
    for t in [p[0] for p in robot_poses]:
        while cidx + 1 < len(contacts) and contacts[cidx + 1][0] <= t:
            cidx += 1
        cstates.append(contact_state(contacts[cidx][1]))

    # XY trajectory
    fig, ax = plt.subplots(figsize=(6, 6))
    stance_mask = [(stance_start <= t + t0 <= stance_end) for t in times]
    trot_mask = [(trot_start <= t + t0 <= trot_end) for t in times]
    ax.plot([xs[i] for i in range(len(xs)) if stance_mask[i]],
            [ys[i] for i in range(len(ys)) if stance_mask[i]],
            'g-', lw=1.5, label='Stance baseline')
    ax.plot([xs[i] for i in range(len(xs)) if trot_mask[i]],
            [ys[i] for i in range(len(ys)) if trot_mask[i]],
            'b-', lw=1.5, label='Zero-velocity trot')
    ax.plot(xs[0], ys[0], 'go', markersize=8, label='Start')
    ax.plot(xs[-1], ys[-1], 'ro', markersize=8, label='End')
    ax.set_xlabel('World x [m]')
    ax.set_ylabel('World y [m]')
    ax.set_title('SD05 base horizontal trajectory')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(f'{args.output_dir}/sd05_zero_trot_xy_trajectory.png', dpi=150)
    plt.close(fig)

    # Time series: x, y, yaw
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, data, label in zip(axs, [xs, ys, yaws], ['x [m]', 'y [m]', 'yaw [deg]']):
        ax.plot(times, data, 'b-', lw=0.8)
        ax.axvspan(stance_start - t0, stance_end - t0, color='green', alpha=0.15, label='Stance baseline')
        ax.axvspan(trot_start - t0, trot_end - t0, color='blue', alpha=0.15, label='Trot')
        ax.set_ylabel(label)
        ax.grid(True)
    axs[0].legend(loc='upper left')
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('SD05 world pose vs time')
    fig.tight_layout()
    fig.savefig(f'{args.output_dir}/sd05_zero_trot_pose_time.png', dpi=150)
    plt.close(fig)

    # Roll and pitch
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, rolls, 'r-', lw=0.8, label='Roll')
    ax.plot(times, pitches, 'g-', lw=0.8, label='Pitch')
    ax.axvspan(trot_start - t0, trot_end - t0, color='blue', alpha=0.15, label='Trot')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('SD05 base roll and pitch')
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(f'{args.output_dir}/sd05_zero_trot_roll_pitch.png', dpi=150)
    plt.close(fig)

    # Contact state
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.plot(times, cstates, 'k-', lw=0.8)
    ax.axvspan(trot_start - t0, trot_end - t0, color='blue', alpha=0.15)
    ax.set_yticks([-1, 0, 1, 2, 3])
    ax.set_yticklabels(['FR+RL', 'Flight', 'FL+RR', '3+ legs', 'Other'])
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Contact state')
    ax.set_title('Ground contact state (FL+RR / FR+RL diagonal support phases)')
    ax.set_ylim(-1.5, 3.5)
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(f'{args.output_dir}/sd05_zero_trot_contact_state.png', dpi=150)
    plt.close(fig)

    # Trot displacement accumulation (relative to trot start)
    trot_indices = [i for i, tp in enumerate([p[0] for p in robot_poses]) if trot_start <= tp <= trot_end]
    if trot_indices:
        trot_times = [times[i] - (trot_start - t0) for i in trot_indices]
        x0, y0, yaw0 = xs[trot_indices[0]], ys[trot_indices[0]], yaws[trot_indices[0]]
        dxs = [xs[i] - x0 for i in trot_indices]
        dys = [ys[i] - y0 for i in trot_indices]
        dyaws = [yaws[i] - yaw0 for i in trot_indices]

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(trot_times, [d * 1000 for d in dxs], 'r-', lw=1.2, label='Δx (backward+)')
        ax.plot(trot_times, [d * 1000 for d in dys], 'g-', lw=1.2, label='Δy (right+)')
        ax.plot(trot_times, dyaws, 'b-', lw=1.2, label='Δyaw')
        ax.axhline(0, color='k', ls='--', lw=0.5)
        ax.set_xlabel('Time since trot start [s]')
        ax.set_ylabel('Δx / Δy [mm] or Δyaw [deg]')
        ax.set_title('Cumulative displacement during zero-velocity trot')
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(f'{args.output_dir}/sd05_zero_trot_displacement_accumulation.png', dpi=150)
        plt.close(fig)

    # Diagonal support phase average velocities (bar chart)
    def phase_average_velocity(active_feet):
        cidx = 0
        vx_sum, vy_sum, yawrate_sum, n = 0.0, 0.0, 0.0, 0
        prev = None
        for i, t in enumerate([p[0] for p in robot_poses]):
            if t < trot_start or t > trot_end:
                continue
            while cidx + 1 < len(contacts) and contacts[cidx + 1][0] <= t:
                cidx += 1
            _, feet = contacts[cidx]
            if feet == active_feet:
                state = (xs[i], ys[i], math.radians(yaws[i]))
                if prev is not None:
                    dt = t - prev[0]
                    if dt > 1e-6:
                        vx_sum += (state[0] - prev[1][0]) / dt
                        vy_sum += (state[1] - prev[1][1]) / dt
                        yawrate_sum += yaw_delta(prev[1][2], state[2]) / dt
                        n += 1
                prev = (t, state)
        return (vx_sum / n, vy_sum / n, yawrate_sum / n) if n else (0, 0, 0)

    fl_rr = phase_average_velocity({'FL', 'RR'})
    fr_rl = phase_average_velocity({'FR', 'RL'})

    fig, ax = plt.subplots(figsize=(8, 5))
    x_labels = ['vx [m/s]', 'vy [m/s]', 'yaw rate [rad/s]']
    x_pos = range(len(x_labels))
    width = 0.35
    ax.bar([p - width / 2 for p in x_pos], fl_rr, width, label='FL+RR')
    ax.bar([p + width / 2 for p in x_pos], fr_rl, width, label='FR+RL')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.axhline(0, color='k', ls='-', lw=0.5)
    ax.set_ylabel('Average velocity')
    ax.set_title('Average base velocity per diagonal support phase')
    ax.legend()
    ax.grid(True, axis='y')
    fig.tight_layout()
    fig.savefig(f'{args.output_dir}/sd05_zero_trot_phase_velocity_bars.png', dpi=150)
    plt.close(fig)

    print(f'Plots saved to {args.output_dir}/')


if __name__ == '__main__':
    main()
