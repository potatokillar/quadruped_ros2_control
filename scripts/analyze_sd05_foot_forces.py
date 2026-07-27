#!/usr/bin/env python3
"""Analyze SD05 zero-velocity trot drift: ground-truth velocity, Kalman bias,
and per-foot ground reaction forces from the diagnostic contact-pad wrenches."""

import argparse
import math
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from ros_gz_interfaces.msg import Contacts
from control_input_msgs.msg import Inputs

FEET = ('FL', 'FR', 'RL', 'RR')


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_bag(path):
    poses, odoms, wrenches, contacts, inputs = [], [], defaultdict(list), [], []
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == '/world/empty/pose/info':
            msg = deserialize_message(data, TFMessage)
            for tr in msg.transforms:
                if tr.child_frame_id == 'robot':
                    p = tr.transform.translation
                    q = tr.transform.rotation
                    poses.append((t, p.x, p.y, p.z, yaw_from_quat(q.x, q.y, q.z, q.w)))
        elif topic == '/odom':
            msg = deserialize_message(data, Odometry)
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            odoms.append((t, v.x, v.y, v.z, w.z))
        elif topic.startswith('/diagnostics/foot_contact_wrenches/'):
            foot = topic.rsplit('/', 1)[1]
            msg = deserialize_message(data, WrenchStamped)
            f = msg.wrench.force
            wrenches[foot].append((t, f.x, f.y, f.z, msg.header.frame_id))
        elif topic == '/ground_contacts':
            msg = deserialize_message(data, Contacts)
            feet = set()
            for c in msg.contacts:
                name = c.collision2.name if 'ground_plane' in c.collision1.name else c.collision1.name
                for cand in ('FL', 'FR', 'RL', 'RR'):
                    if cand + '_foot' in name or cand + '_contact_pad' in name:
                        feet.add(cand)
                        break
            contacts.append((t, feet))
        elif topic == '/control_input':
            msg = deserialize_message(data, Inputs)
            inputs.append((t, msg.command))
    return poses, odoms, wrenches, contacts, inputs


def trot_window(inputs):
    by_cmd = defaultdict(list)
    for t, cmd in inputs:
        by_cmd[cmd].append(t)
    cmd3_end = max(by_cmd[3])
    after = [t for t in by_cmd[2] if t > cmd3_end]
    return cmd3_end, min(after) if after else max(by_cmd[2])


def truth_base_velocity(poses, t0, t1):
    """Mean base-frame velocity from world poses inside [t0, t1]."""
    window = [p for p in poses if t0 <= p[0] <= t1]
    vx_w, vy_w, yaw_rates = [], [], []
    for (ta, xa, ya, _, yawa), (tb, xb, yb, _, yawb) in zip(window, window[1:]):
        dt = tb - ta
        if dt <= 0:
            continue
        vx_w.append((xb - xa) / dt)
        vy_w.append((yb - ya) / dt)
        dyaw = math.atan2(math.sin(yawb - yawa), math.cos(yawb - yawa))
        yaw_rates.append(dyaw / dt)
    n = len(vx_w)
    mvx = sum(vx_w) / n
    mvy = sum(vy_w) / n
    myaw = sum(yaw_rates) / n
    # Rotate mean world velocity into the base frame using mean yaw.
    mean_yaw = sum(p[4] for p in window) / len(window)
    c, s = math.cos(mean_yaw), math.sin(mean_yaw)
    return mvx, mvy, myaw, c * mvx + s * mvy, -s * mvx + c * mvy


def mean_odom(odoms, t0, t1):
    w = [o for o in odoms if t0 <= o[0] <= t1]
    n = len(w)
    return tuple(sum(o[i] for o in w) / n for i in range(1, 5))


def mean_wrench(samples, t0, t1):
    w = [s for s in samples if t0 <= s[0] <= t1]
    if not w:
        return None
    n = len(w)
    return tuple(sum(s[i] for s in w) / n for i in range(1, 4)), len(w), w[0][4]


def contact_phases(contacts, t0, t1):
    """Split trot window into FL+RR and FR+RL segments."""
    segs = {'FLRR': [], 'FRRL': []}
    for t, feet in contacts:
        if not (t0 <= t <= t1):
            continue
        if feet == {'FL', 'RR'}:
            segs['FLRR'].append(t)
        elif feet == {'FR', 'RL'}:
            segs['FRRL'].append(t)
    return {k: (min(v), max(v)) for k, v in segs.items() if v}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    args = parser.parse_args()

    poses, odoms, wrenches, contacts, inputs = load_bag(args.bag)
    t0, t1 = trot_window(inputs)
    print(f'Trot window: {t0:.3f} .. {t1:.3f} ({t1 - t0:.1f} s)')

    mvx, mvy, myaw, bvx, bvy = truth_base_velocity(poses, t0, t1)
    print(f'\n=== Gazebo truth velocity (trot mean) ===')
    print(f'world: vx={mvx:+.5f} vy={mvy:+.5f} m/s, yaw_rate={myaw:+.5f} rad/s')
    print(f'base:  vx={bvx:+.5f} vy={bvy:+.5f} m/s')

    ovx, ovy, ovz, oyaw = mean_odom(odoms, t0, t1)
    print(f'\n=== Kalman /odom twist (trot mean, base frame) ===')
    print(f'vx={ovx:+.5f} vy={ovy:+.5f} vz={ovz:+.5f} m/s, yaw_rate={oyaw:+.5f} rad/s')
    print(f'Kalman bias (odom - truth, base): dvx={ovx - bvx:+.5f} dvy={ovy - bvy:+.5f} m/s')

    print(f'\n=== Per-foot contact-pad wrench (trot mean) ===')
    for foot in FEET:
        res = mean_wrench(wrenches[foot], t0, t1)
        if res is None:
            print(f'{foot}: no samples')
            continue
        (fx, fy, fz), n, frame = res
        print(f'{foot}: fx={fx:+8.3f} fy={fy:+8.3f} fz={fz:+8.3f} N  (n={n}, frame={frame!r})')

    print(f'\n=== Per-foot wrench per diagonal support phase ===')
    for seg_name, (s0, s1) in contact_phases(contacts, t0, t1).items():
        print(f'-- {seg_name} segments overall span {s0:.3f}..{s1:.3f}')
        # Use per-sample phase membership instead of the whole span.
        phase_times = {'FLRR': set(), 'FRRL': set()}
        for t, feet in contacts:
            if t0 <= t <= t1:
                if feet == {'FL', 'RR'}:
                    phase_times['FLRR'].add(round(t, 3))
                elif feet == {'FR', 'RL'}:
                    phase_times['FRRL'].add(round(t, 3))
        for foot in FEET:
            sel = [s for s in wrenches[foot] if t0 <= s[0] <= t1
                   and round(s[0], 3) in phase_times[seg_name]]
            if not sel:
                print(f'  {foot}: no samples in phase')
                continue
            n = len(sel)
            fx = sum(s[1] for s in sel) / n
            fy = sum(s[2] for s in sel) / n
            fz = sum(s[3] for s in sel) / n
            print(f'  {foot}: fx={fx:+8.3f} fy={fy:+8.3f} fz={fz:+8.3f} N  (n={n})')


if __name__ == '__main__':
    main()
