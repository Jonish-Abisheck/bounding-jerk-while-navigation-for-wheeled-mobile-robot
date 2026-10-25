#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under the Apache License, Version 2.0
#
# Baseline Jerk Logger — records velocity, acceleration, and jerk
# directly from the planner (no jerk_reducer). Saves to CSV in the
# same schema as jerk_data_logger.py for direct comparison.

import rospy
import csv
import os
import threading
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class BaselineJerkLogger:
    def __init__(self):
        rospy.init_node('baseline_jerk_logger', anonymous=True)

        self.lock = threading.Lock()

        # ── Parameters ────────────────────────────────────────────────
        self.output_dir   = rospy.get_param('~output_dir',   os.path.expanduser('~/jerk_logs'))
        self.file_prefix  = rospy.get_param('~file_prefix',  'baseline_log')
        self.log_odom     = rospy.get_param('~log_odom',     True)
        self.log_rate     = rospy.get_param('~log_rate',     20.0)

        self.cmd_topic    = rospy.get_param('~cmd_topic',    '/cmd_vel')
        self.odom_topic   = rospy.get_param('~odom_topic',   '/odometry/filtered')

        # ── CSV setup ─────────────────────────────────────────────────
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir, f'{self.file_prefix}_{stamp}.csv')

        # ── Latest message cache ──────────────────────────────────────
        self.cmd_vel = None
        self.odom    = None

        # ── Derivative state (cmd-based) ──────────────────────────────
        self.prev_time     = None
        self.prev_vx       = None
        self.prev_vy       = None
        self.prev_wz       = None
        self.prev_accel_vx = None
        self.prev_accel_vy = None
        self.prev_accel_wz = None

        # ── Derivative state (odom-based) ─────────────────────────────
        self.prev_odom_time     = None
        self.prev_odom_vx       = None
        self.prev_odom_vy       = None
        self.prev_odom_wz       = None
        self.prev_odom_accel_vx = None
        self.prev_odom_accel_vy = None
        self.prev_odom_accel_wz = None

        # ── CSV columns ───────────────────────────────────────────────
        self.fieldnames = [
            'time_sec',

            # Raw planner command
            'cmd_vx', 'cmd_vy', 'cmd_vz',
            'cmd_wx', 'cmd_wy', 'cmd_wz',

            # Derivatives from command signal
            'cmd_accel_vx', 'cmd_accel_vy', 'cmd_accel_wz',
            'cmd_jerk_vx',  'cmd_jerk_vy',  'cmd_jerk_wz',

            # Body-frame jerk (tangential / lateral / angular)
            'cmd_jerk_tan', 'cmd_jerk_lat', 'cmd_jerk_ang',

            # Odometry velocities
            'odom_vx', 'odom_vy', 'odom_vz',
            'odom_wx', 'odom_wy', 'odom_wz',

            # Derivatives from odometry signal
            'odom_accel_vx', 'odom_accel_vy', 'odom_accel_wz',
            'odom_jerk_vx',  'odom_jerk_vy',  'odom_jerk_wz',

            # Body-frame jerk from odom
            'odom_jerk_tan', 'odom_jerk_lat', 'odom_jerk_ang',
        ]

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer   = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()

        # ── Subscribers ───────────────────────────────────────────────
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_cb, queue_size=1)
        if self.log_odom:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        # ── Timer ─────────────────────────────────────────────────────
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.log_rate), self.timer_cb)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('[BaselineJerkLogger] Logging to %s', self.csv_path)
        rospy.loginfo('[BaselineJerkLogger] cmd_topic=%s  odom_topic=%s  rate=%.1f Hz',
                      self.cmd_topic, self.odom_topic, self.log_rate)

    # ── Callbacks ─────────────────────────────────────────────────────

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd_vel = msg

    def odom_cb(self, msg):
        with self.lock:
            self.odom = msg

    # ── Helpers ───────────────────────────────────────────────────────

    def _get_twist(self, msg):
        if msg is None:
            return dict(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0)
        return dict(
            lx=msg.linear.x,  ly=msg.linear.y,  lz=msg.linear.z,
            ax=msg.angular.x, ay=msg.angular.y, az=msg.angular.z)

    def _get_odom_twist(self, msg):
        if msg is None:
            return dict(lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0)
        t = msg.twist.twist
        return dict(
            lx=t.linear.x,  ly=t.linear.y,  lz=t.linear.z,
            ax=t.angular.x, ay=t.angular.y, az=t.angular.z)

    def _body_jerk(self, vx, vy, wz,
                   accel_vx, accel_vy, accel_wz,
                   jerk_vx, jerk_vy, jerk_wz):
        """
        Tangential jerk : dv̇/dt − v·ω²
        Lateral jerk    : 2·v̇·ω + v·ω̇
        Angular jerk    : ω̈
        """
        j_tan = jerk_vx - vx * wz * wz
        j_lat = 2.0 * accel_vx * wz + vx * accel_wz
        j_ang = jerk_wz
        return j_tan, j_lat, j_ang

    def _differentiate(self, now, vx, vy, wz,
                        prev_time,
                        prev_vx, prev_vy, prev_wz,
                        prev_accel_vx, prev_accel_vy, prev_accel_wz):
        zero6 = (0.0,) * 6

        if prev_time is None:
            return zero6, now, vx, vy, wz, None, None, None

        dt = now - prev_time
        if dt <= 1e-9:
            a_vx = prev_accel_vx if prev_accel_vx is not None else 0.0
            a_vy = prev_accel_vy if prev_accel_vy is not None else 0.0
            a_wz = prev_accel_wz if prev_accel_wz is not None else 0.0
            return (a_vx, a_vy, a_wz, 0.0, 0.0, 0.0), \
                   prev_time, prev_vx, prev_vy, prev_wz, \
                   prev_accel_vx, prev_accel_vy, prev_accel_wz

        a_vx = (vx - prev_vx) / dt
        a_vy = (vy - prev_vy) / dt
        a_wz = (wz - prev_wz) / dt

        if prev_accel_vx is None:
            j_vx = j_vy = j_wz = 0.0
        else:
            j_vx = (a_vx - prev_accel_vx) / dt
            j_vy = (a_vy - prev_accel_vy) / dt
            j_wz = (a_wz - prev_accel_wz) / dt

        return (a_vx, a_vy, a_wz, j_vx, j_vy, j_wz), \
               now, vx, vy, wz, a_vx, a_vy, a_wz

    # ── Timer callback ────────────────────────────────────────────────

    def timer_cb(self, _event):
        now = rospy.Time.now().to_sec()

        with self.lock:
            cmd  = self._get_twist(self.cmd_vel)
            odom = self._get_odom_twist(self.odom)

        # Derivatives from command
        (ca_vx, ca_vy, ca_wz,
         cj_vx, cj_vy, cj_wz), \
        self.prev_time, \
        self.prev_vx, self.prev_vy, self.prev_wz, \
        self.prev_accel_vx, self.prev_accel_vy, self.prev_accel_wz = \
            self._differentiate(
                now, cmd['lx'], cmd['ly'], cmd['az'],
                self.prev_time,
                self.prev_vx, self.prev_vy, self.prev_wz,
                self.prev_accel_vx, self.prev_accel_vy, self.prev_accel_wz)

        cj_tan, cj_lat, cj_ang = self._body_jerk(
            cmd['lx'], cmd['ly'], cmd['az'],
            ca_vx, ca_vy, ca_wz,
            cj_vx, cj_vy, cj_wz)

        # Derivatives from odom
        (oa_vx, oa_vy, oa_wz,
         oj_vx, oj_vy, oj_wz), \
        self.prev_odom_time, \
        self.prev_odom_vx, self.prev_odom_vy, self.prev_odom_wz, \
        self.prev_odom_accel_vx, self.prev_odom_accel_vy, self.prev_odom_accel_wz = \
            self._differentiate(
                now, odom['lx'], odom['ly'], odom['az'],
                self.prev_odom_time,
                self.prev_odom_vx, self.prev_odom_vy, self.prev_odom_wz,
                self.prev_odom_accel_vx, self.prev_odom_accel_vy, self.prev_odom_accel_wz)

        oj_tan, oj_lat, oj_ang = self._body_jerk(
            odom['lx'], odom['ly'], odom['az'],
            oa_vx, oa_vy, oa_wz,
            oj_vx, oj_vy, oj_wz)

        row = {
            'time_sec': round(now, 6),

            'cmd_vx': round(cmd['lx'], 6),
            'cmd_vy': round(cmd['ly'], 6),
            'cmd_vz': round(cmd['lz'], 6),
            'cmd_wx': round(cmd['ax'], 6),
            'cmd_wy': round(cmd['ay'], 6),
            'cmd_wz': round(cmd['az'], 6),

            'cmd_accel_vx': round(ca_vx, 6),
            'cmd_accel_vy': round(ca_vy, 6),
            'cmd_accel_wz': round(ca_wz, 6),
            'cmd_jerk_vx':  round(cj_vx, 6),
            'cmd_jerk_vy':  round(cj_vy, 6),
            'cmd_jerk_wz':  round(cj_wz, 6),

            'cmd_jerk_tan': round(cj_tan, 6),
            'cmd_jerk_lat': round(cj_lat, 6),
            'cmd_jerk_ang': round(cj_ang, 6),

            'odom_vx': round(odom['lx'], 6),
            'odom_vy': round(odom['ly'], 6),
            'odom_vz': round(odom['lz'], 6),
            'odom_wx': round(odom['ax'], 6),
            'odom_wy': round(odom['ay'], 6),
            'odom_wz': round(odom['az'], 6),

            'odom_accel_vx': round(oa_vx, 6),
            'odom_accel_vy': round(oa_vy, 6),
            'odom_accel_wz': round(oa_wz, 6),
            'odom_jerk_vx':  round(oj_vx, 6),
            'odom_jerk_vy':  round(oj_vy, 6),
            'odom_jerk_wz':  round(oj_wz, 6),

            'odom_jerk_tan': round(oj_tan, 6),
            'odom_jerk_lat': round(oj_lat, 6),
            'odom_jerk_ang': round(oj_ang, 6),
        }

        self.writer.writerow(row)

    # ── Shutdown ──────────────────────────────────────────────────────

    def on_shutdown(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.loginfo('[BaselineJerkLogger] CSV saved: %s', self.csv_path)
        except Exception:
            pass


if __name__ == '__main__':
    try:
        logger = BaselineJerkLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass