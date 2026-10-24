#!/usr/bin/env python3
"""Putar ulang rekaman perintah gerak dari record_gerak.py.

Tidak memakai odometri sama sekali: perintah diberikan persis pada waktu
simulasi yang sama dengan saat merekam.
Pemakaian: putar_ulang.py [file_rekaman.json]
"""
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


# Harus sama dengan record_gerak.py.
# False = pakai jam komputer (tidak butuh /clock, aman untuk aturan tugas).
# True  = pakai waktu simulasi, hanya kalau topic /clock memang tersedia.
USE_SIM_TIME = False

CONTROL_PERIOD = 0.05
STATUS_LOG_PERIOD = 1.0
CLOCK_STALL_WARN = 3.0

# Tunggu koneksi /cmd_vel siap sebelum waktu mulai dihitung, supaya
# perintah pertama tidak hilang.
START_DELAY = 1.0       # detik tambahan setelah penerima terdeteksi
READY_TIMEOUT = 5.0     # detik maksimal menunggu penerima /cmd_vel

DEFAULT_FILE = "rekaman_gerak.json"


class MotionReplayer(Node):
    def __init__(self, data):
        super().__init__(
            "motion_replayer",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, USE_SIM_TIME)
            ],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.segments = data["segments"]
        self.markers = data["markers"]
        self.duration = float(data["duration"])

        self.seg_index = -1
        self.marker_index = 0
        self.linear = 0.0
        self.angular = 0.0

        self.t0 = None
        self.ready_since = None
        self.created_wall = time.monotonic()
        self.finished = False
        self.last_status_wall = 0.0

        self.last_sim = None
        self.last_sim_change = time.monotonic()
        self.warned_clock = False

        wall = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(
            CONTROL_PERIOD, self.tick, clock=wall
        )

        self.get_logger().info(
            "Rekaman: {} perintah, {} titik, durasi {:.1f} s".format(
                len(self.segments), len(self.markers), self.duration
            )
        )

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def check_clock(self, sim_now):
        wall_now = time.monotonic()
        if sim_now != self.last_sim:
            self.last_sim = sim_now
            self.last_sim_change = wall_now
            self.warned_clock = False
        elif (
            USE_SIM_TIME
            and not self.warned_clock
            and wall_now - self.last_sim_change > CLOCK_STALL_WARN
        ):
            self.get_logger().warning(
                "Waktu simulasi tidak bergerak. Cek /clock "
                "atau Gazebo sedang pause."
            )
            self.warned_clock = True

    def tick(self):
        if self.finished:
            self.publish(0.0, 0.0)
            return

        sim_now = self.sim_time()
        self.check_clock(sim_now)

        # Tunggu sampai /clock benar-benar mulai berjalan.
        if USE_SIM_TIME and sim_now <= 0.0:
            self.publish(0.0, 0.0)
            return

        if self.t0 is None:
            self.publish(0.0, 0.0)
            wall_now = time.monotonic()

            if self.ready_since is None:
                if self.cmd_pub.get_subscription_count() > 0:
                    self.ready_since = wall_now
                elif wall_now - self.created_wall > READY_TIMEOUT:
                    self.get_logger().warning(
                        "Tidak terdeteksi penerima /cmd_vel; tetap dimulai."
                    )
                    self.ready_since = wall_now
                return

            if wall_now - self.ready_since < START_DELAY:
                return

            self.t0 = sim_now
            self.get_logger().info("Mulai memutar ulang.")

        t = sim_now - self.t0

        while (
            self.seg_index + 1 < len(self.segments)
            and self.segments[self.seg_index + 1]["t"] <= t
        ):
            self.seg_index += 1
            seg = self.segments[self.seg_index]
            self.linear = seg["linear"]
            self.angular = seg["angular"]

        while (
            self.marker_index < len(self.markers)
            and self.markers[self.marker_index]["t"] <= t
        ):
            self.get_logger().info(
                "===== Titik {}/{} terlewati (t={:.1f} s) =====".format(
                    self.markers[self.marker_index]["n"],
                    len(self.markers),
                    t,
                )
            )
            self.marker_index += 1

        wall_now = time.monotonic()
        if wall_now - self.last_status_wall >= STATUS_LOG_PERIOD:
            if self.marker_index < len(self.markers):
                target = "menuju titik {}/{}".format(
                    self.markers[self.marker_index]["n"], len(self.markers)
                )
            else:
                target = "setelah titik terakhir"
            self.get_logger().info(
                "t={:.1f}/{:.1f} s | {} | v={:.2f} m/s, w={:.2f} rad/s".format(
                    t, self.duration, target, self.linear, self.angular
                )
            )
            self.last_status_wall = wall_now

        if t >= self.duration:
            self.publish(0.0, 0.0)
            self.finished = True
            self.get_logger().info("Putar ulang selesai. Robot berhenti.")
            self.get_logger().info(
                "WAKTU TEMPUH: {:.2f} detik ({})".format(
                    t,
                    "waktu simulasi" if USE_SIM_TIME else "jam komputer",
                )
            )
            return

        self.publish(self.linear, self.angular)


def main(args=None):
    cli_args = remove_ros_args(sys.argv)[1:]
    path = cli_args[0] if cli_args else DEFAULT_FILE
    path = os.path.abspath(path)

    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print("Gagal membaca {}: {}".format(path, e))
        return

    print("File rekaman: {}".format(path))
    if data.get("use_sim_time") != USE_SIM_TIME:
        print(
            "PERINGATAN: rekaman dibuat dengan use_sim_time={}, tapi sekarang "
            "USE_SIM_TIME={}. Rekam ulang agar waktunya cocok.".format(
                data.get("use_sim_time"), USE_SIM_TIME
            )
        )
    input("Pastikan robot di posisi awal yang sama. Tekan Enter untuk mulai.")

    # Ctrl+C jadi KeyboardInterrupt biasa, sehingga perintah berhenti
    # di bagian finally masih sempat terkirim.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionReplayer(data)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
