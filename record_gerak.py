#!/usr/bin/env python3
"""Rekam perintah gerak (cmd_vel) dari keyboard beserta waktunya.

Yang disimpan bukan posisi odometri, tapi urutan perintah dan kapan
perintah itu diberikan (dalam waktu simulasi). Hasilnya diputar ulang
oleh putar_ulang.py.
"""
import json
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


# False = pakai jam komputer (tidak butuh /clock, aman untuk aturan tugas).
# True  = pakai waktu simulasi, hanya kalau topic /clock memang tersedia.
# Harus sama dengan putar_ulang.py.
USE_SIM_TIME = False

CONTROL_PERIOD = 0.05
LINEAR_SPEED = 0.3      # m/s, kecepatan awal
TURN_SPEED = 0.5        # rad/s
SPEED_STEP = 0.05
MAX_SPEED = 1.0
CLOCK_STALL_WARN = 3.0  # detik (jam komputer) tanpa /clock bergerak

OUTPUT_FILE = "rekaman_gerak.json"


class MotionRecorder(Node):
    def __init__(self):
        super().__init__(
            "motion_recorder",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, USE_SIM_TIME)
            ],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.linear_speed = LINEAR_SPEED
        self.linear = 0.0
        self.angular = 0.0

        self.t0 = None          # waktu simulasi saat gerakan pertama
        self.segments = []      # {"t", "linear", "angular"}
        self.markers = []       # {"t", "n"}
        self.done = False

        self.last_sim = None
        self.last_sim_change = time.monotonic()
        self.warned_clock = False

        # Timer memakai jam komputer supaya tetap jalan walau /clock mati.
        wall = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(
            CONTROL_PERIOD, self.tick, clock=wall
        )

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_clock(self):
        sim_now = self.sim_time()
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
                "(ros2 topic echo /clock) atau Gazebo sedang pause."
            )
            self.warned_clock = True

    def set_command(self, linear, angular):
        now = self.sim_time()
        if self.t0 is None:
            if linear == 0.0 and angular == 0.0:
                return  # belum mulai, abaikan perintah berhenti
            self.t0 = now

        t = now - self.t0
        if (linear, angular) != (self.linear, self.angular):
            if self.segments:
                print("   (perintah sebelumnya berlangsung {:.1f} s)".format(
                    t - self.segments[-1]["t"]))
            self.segments.append(
                {"t": round(t, 3), "linear": linear, "angular": angular}
            )
        self.linear = linear
        self.angular = angular

    def handle_key(self, key):
        if key == "w":
            self.set_command(self.linear_speed, 0.0)
            print("Maju ({} m/s)".format(self.linear_speed))
        elif key == "s":
            self.set_command(-self.linear_speed, 0.0)
            print("Mundur ({} m/s)".format(self.linear_speed))
        elif key == "a":
            self.set_command(0.0, TURN_SPEED)
            print("Putar kiri")
        elif key == "d":
            self.set_command(0.0, -TURN_SPEED)
            print("Putar kanan")
        elif key == " ":
            self.set_command(0.0, 0.0)
            print("Berhenti")
        elif key in ("+", "=", "-"):
            step = SPEED_STEP if key in ("+", "=") else -SPEED_STEP
            self.linear_speed = round(
                min(MAX_SPEED, max(SPEED_STEP, self.linear_speed + step)), 2
            )
            if self.linear > 0:
                self.set_command(self.linear_speed, 0.0)
            elif self.linear < 0:
                self.set_command(-self.linear_speed, 0.0)
            print("Kecepatan maju: {} m/s".format(self.linear_speed))
        elif key == "p":
            if self.t0 is None:
                print("Gerakkan robot dulu sebelum menandai titik.")
            else:
                t = round(self.sim_time() - self.t0, 3)
                n = len(self.markers) + 1
                self.markers.append({"t": t, "n": n})
                print("Titik {} ditandai pada t={:.1f} s".format(n, t))
        elif key == "q":
            self.set_command(0.0, 0.0)
            self.done = True
            print("Selesai merekam.")

    def tick(self):
        self.check_clock()

        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(sys.stdin.fileno(), 1024).decode(errors="ignore")
            if not data:
                break
            if data.startswith("\x1b"):
                continue  # abaikan tombol panah dan tombol khusus
            for key in data.lower():
                self.handle_key(key)
                if self.done:
                    break
            if self.done:
                break

        self.publish(self.linear, self.angular)

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def save(self, filename):
        times = [s["t"] for s in self.segments]
        times += [m["t"] for m in self.markers]
        data = {
            "use_sim_time": USE_SIM_TIME,
            "duration": max(times) if times else 0.0,
            "segments": self.segments,
            "markers": self.markers,
        }
        path = os.path.join(os.getcwd(), filename)
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            print("\nRekaman tersimpan di: {}".format(path))
            print(
                "{} perintah, {} titik, durasi {:.1f} s".format(
                    len(self.segments), len(self.markers), data["duration"]
                )
            )
        except OSError as e:
            print("\nGagal menyimpan file: {}".format(e))


def main(args=None):
    cli_args = remove_ros_args(sys.argv)[1:]
    filename = cli_args[0] if cli_args else OUTPUT_FILE

    # Ctrl+C jadi KeyboardInterrupt biasa, sehingga perintah berhenti
    # di bagian finally masih sempat terkirim.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionRecorder()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        print("\nPastikan robot ada di posisi awal (reset Gazebo dulu).")
        print("Kontrol:")
        print("  w/s = maju/mundur, a/d = putar kiri/kanan, spasi = berhenti")
        print("  + / - = ubah kecepatan maju (sekarang {} m/s)".format(
            node.linear_speed))
        print("  p = tandai titik (hanya untuk penanda saat diputar ulang)")
        print("  q = selesai dan simpan\n")

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass
    finally:
        node.set_command(0.0, 0.0)
        node.publish(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.save(filename)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
