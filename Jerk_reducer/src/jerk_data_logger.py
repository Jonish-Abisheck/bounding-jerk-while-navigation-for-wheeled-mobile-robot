#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under the Apache License, Version 2.0

import csv
import math
import os
import threading
from datetime import datetime

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

NAN = float('nan')


def new_state():
    return {'t': None, 'v': 0.0, 'vy': 0.0, 'w': 0.0, 'a': None, 'ay': None, 'al': None}


def step(state, t, v, vy, w, max_gap):
    """Finite-difference accel/jerk using current (k) v, w in the nonlinear terms."""
    out = {'dt': NAN, 'a': NAN, 'ay': NAN, 'al': NAN,
           'jt': NAN, 'jt_prev': NAN, 'jl': NAN, 'ja': NAN}
    if state['t'] is not None:
        dt = t - state['t']
        if dt < 0.0 or dt > max_gap:
            state.update(t=t, v=v, vy=vy, w=w, a=None, ay=None, al=None)
            return out
        if dt <= 1e-6:
            return out
        a = (v - state['v']) / dt
        ay = (vy - state['vy']) / dt
        al = (w - state['w']) / dt
        out.update(dt=dt, a=a, ay=ay, al=al)
        if state['a'] is not None:
            vddot = (a - state['a']) / dt
            out['jt'] = vddot - v * w * w
            out['jt_prev'] = vddot - state['v'] * state['w'] ** 2
            out['jl'] = 2.0 * a * w + v * al
            out['ja'] = (al - state['al']) / dt
        state.update(a=a, ay=ay, al=al)
    state.update(t=t, v=v, vy=vy, w=w)
    return out


class JerkDataLogger:
    def __init__(self):
        rospy.init_node('jerk_data_logger', anonymous=True)
        self.lock = threading.Lock()

        self.output_dir  = rospy.get_param('~output_dir', os.path.expanduser('~/catkin_ws/src/jerk_reducer/jerk_logs'))
        self.file_prefix = rospy.get_param('~file_prefix', 'jerk_log')
        self.log_odom    = rospy.get_param('~log_odom', True)
        self.max_gap     = rospy.get_param('~max_gap', 0.25)
        self.flush_every = rospy.get_param('~flush_every', 20)

        self.raw_cmd_topic      = rospy.get_param('~raw_cmd_topic',      '/cmd_vel_raw')
        self.filtered_cmd_topic = rospy.get_param('~filtered_cmd_topic', '/cmd_vel')
        self.raw_jerk_topic     = rospy.get_param('~raw_jerk_topic',     '/cmd_jerk_raw')
        self.limited_jerk_topic = rospy.get_param('~limited_jerk_topic', '/cmd_jerk_limited')
        self.odom_topic         = rospy.get_param('~odom_topic',         '/odometry/filtered')

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'{self.file_prefix}_{stamp}.csv')

        self.cmd_vel_raw = None
        self.cmd_vel_filtered = None
        self.cmd_jerk_raw = None
        self.odom = None

        self.cmd_state = new_state()
        self.odom_state = new_state()
        self.cmd_d = step(new_state(), 0, 0, 0, 0, 1.0)
        self.odom_d = dict(self.cmd_d)

        self.rows = 0
        self.stats = {}

        self.fieldnames = [
            'time_sec', 'dt_cmd',
            'cmd_raw_vx', 'cmd_raw_vy', 'cmd_raw_vz',
            'cmd_raw_wx', 'cmd_raw_wy', 'cmd_raw_wz',
            'cmd_filt_vx', 'cmd_filt_vy', 'cmd_filt_vz',
            'cmd_filt_wx', 'cmd_filt_wy', 'cmd_filt_wz',
            'jerk_raw_tan', 'jerk_raw_lat', 'jerk_raw_ang',
            'jerk_lim_tan', 'jerk_lim_lat', 'jerk_lim_ang',
            'odom_vx', 'odom_vy', 'odom_vz',
            'odom_wx', 'odom_wy', 'odom_wz',
            'accel_cmd_vx', 'accel_cmd_vy', 'accel_cmd_wz',
            'jerk_cmd_tan', 'jerk_cmd_tan_prevform', 'jerk_cmd_lat', 'jerk_cmd_ang',
            'odom_accel_vx', 'odom_accel_wz',
            'odom_jerk_tan', 'odom_jerk_lat', 'odom_jerk_ang',
        ]
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()

        rospy.Subscriber(self.raw_cmd_topic,      Twist, self.raw_cmd_cb,      queue_size=100)
        rospy.Subscriber(self.filtered_cmd_topic, Twist, self.filtered_cmd_cb, queue_size=100)
        rospy.Subscriber(self.raw_jerk_topic,     Twist, self.raw_jerk_cb,     queue_size=100)
        rospy.Subscriber(self.limited_jerk_topic, Twist, self.limited_jerk_cb, queue_size=100)
        if self.log_odom:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=100)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('[JerkDataLogger] Logging to %s', self.csv_path)

    def raw_cmd_cb(self, msg):
        with self.lock:
            self.cmd_vel_raw = msg

    def raw_jerk_cb(self, msg):
        with self.lock:
            self.cmd_jerk_raw = msg

    def filtered_cmd_cb(self, msg):
        t = rospy.Time.now().to_sec()
        with self.lock:
            self.cmd_vel_filtered = msg
            self.cmd_d = step(self.cmd_state, t, msg.linear.x, msg.linear.y,
                              msg.angular.z, self.max_gap)

    def odom_cb(self, msg):
        t = msg.header.stamp.to_sec()
        tw = msg.twist.twist
        with self.lock:
            self.odom = msg
            self.odom_d = step(self.odom_state, t, tw.linear.x, tw.linear.y,
                               tw.angular.z, self.max_gap)

    @staticmethod
    def twist(msg):
        if msg is None:
            return dict(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0)
        return dict(lx=msg.linear.x, ly=msg.linear.y, lz=msg.linear.z,
                    ax=msg.angular.x, ay=msg.angular.y, az=msg.angular.z)

    def track(self, name, val):
        if math.isfinite(val):
            n, ss, mx = self.stats.get(name, (0, 0.0, 0.0))
            self.stats[name] = (n + 1, ss + val * val, max(mx, abs(val)))

    def limited_jerk_cb(self, msg):
        now = rospy.Time.now().to_sec()
        with self.lock:
            raw = self.twist(self.cmd_vel_raw)
            filt = self.twist(self.cmd_vel_filtered)
            rj = self.twist(self.cmd_jerk_raw)
            od = self.twist(None)
            if self.odom is not None:
                t = self.odom.twist.twist
                od = dict(lx=t.linear.x, ly=t.linear.y, lz=t.linear.z,
                          ax=t.angular.x, ay=t.angular.y, az=t.angular.z)
            c, o = self.cmd_d, self.odom_d
            lim = self.twist(msg)

            row = {
                'time_sec': round(now, 6), 'dt_cmd': c['dt'],
                'cmd_raw_vx': raw['lx'], 'cmd_raw_vy': raw['ly'], 'cmd_raw_vz': raw['lz'],
                'cmd_raw_wx': raw['ax'], 'cmd_raw_wy': raw['ay'], 'cmd_raw_wz': raw['az'],
                'cmd_filt_vx': filt['lx'], 'cmd_filt_vy': filt['ly'], 'cmd_filt_vz': filt['lz'],
                'cmd_filt_wx': filt['ax'], 'cmd_filt_wy': filt['ay'], 'cmd_filt_wz': filt['az'],
                'jerk_raw_tan': rj['lx'], 'jerk_raw_lat': rj['ly'], 'jerk_raw_ang': rj['az'],
                'jerk_lim_tan': lim['lx'], 'jerk_lim_lat': lim['ly'], 'jerk_lim_ang': lim['az'],
                'odom_vx': od['lx'], 'odom_vy': od['ly'], 'odom_vz': od['lz'],
                'odom_wx': od['ax'], 'odom_wy': od['ay'], 'odom_wz': od['az'],
                'accel_cmd_vx': c['a'], 'accel_cmd_vy': c['ay'], 'accel_cmd_wz': c['al'],
                'jerk_cmd_tan': c['jt'], 'jerk_cmd_tan_prevform': c['jt_prev'],
                'jerk_cmd_lat': c['jl'], 'jerk_cmd_ang': c['ja'],
                'odom_accel_vx': o['a'], 'odom_accel_wz': o['al'],
                'odom_jerk_tan': o['jt'], 'odom_jerk_lat': o['jl'], 'odom_jerk_ang': o['ja'],
            }
            self.writer.writerow({k: (round(v, 6) if isinstance(v, float) and math.isfinite(v) else v)
                                  for k, v in row.items()})
            for k in ('jerk_lim_tan', 'jerk_lim_lat', 'jerk_cmd_tan', 'jerk_cmd_tan_prevform',
                      'jerk_cmd_lat', 'jerk_cmd_ang', 'odom_jerk_tan', 'odom_jerk_lat'):
                self.track(k, row[k])
            self.rows += 1
            if self.rows % self.flush_every == 0:
                self.csv_file.flush()

    def on_shutdown(self):
        with self.lock:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
            rospy.loginfo('[JerkDataLogger] CSV saved: %s (%d rows)', self.csv_path, self.rows)
            for k, (n, ss, mx) in sorted(self.stats.items()):
                rospy.loginfo('[JerkDataLogger] %-22s RMS=%.3f  max|.|=%.3f  n=%d',
                              k, math.sqrt(ss / n), mx, n)


if __name__ == '__main__':
    try:
        JerkDataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass