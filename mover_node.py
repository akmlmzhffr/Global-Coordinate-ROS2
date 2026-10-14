#!/usr/bin/env python3
"""ROS 2 global-coordinate controller for circular motion using VR and VL.

VR: right-wheel linear velocity in m/s.
VL: left-wheel linear velocity in m/s.
Uses Gazebo /clock, publishes /cmd_vel and /global_coordinate (Pose2D).
Global coordinates are open-loop estimates from commanded motion.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, Pose2D
from rosgraph_msgs.msg import Clock


def read_float(prompt):
    while True:
        raw = input(f"{prompt}: ").strip()
        if not raw:
            print("Input tidak boleh kosong. Masukkan angka.")
            continue
        try:
            value = float(raw)
            if math.isfinite(value):
                return value
        except ValueError:
            pass
        print("Masukkan angka yang valid.")


class GlobalCoordinateCompass(Node):
    def __init__(self, vr, vl, wheel_separation):
        super().__init__("global_coordinate_compass")
        self.set_parameters([
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ])
        self.vr = vr
        self.vl = vl
        self.track = wheel_separation
        self.v = (vr + vl) / 2.0
        self.omega = (vr - vl) / self.track

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_pub = self.create_publisher(Pose2D, "/global_coordinate", 10)
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.timer = self.create_timer(0.05, self.update)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.sim_time = None
        self.last_sim_time = None
        self.last_log_time = -1.0

        self.get_logger().info(
            f"VR={vr:.3f} m/s, VL={vl:.3f} m/s; "
            f"v={self.v:.3f} m/s, omega={self.omega:.3f} rad/s"
        )
        if abs(self.omega) > 1e-9:
            self.get_logger().info(
                f"Perkiraan radius lintasan bertanda R=v/omega={self.v / self.omega:.3f} m"
            )
        else:
            self.get_logger().info("VR=VL: robot bergerak lurus, tidak membentuk lingkaran.")

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def update(self):
        # Use Gazebo simulation time rather than laptop wall-clock time.
        if self.sim_time is None:
            return
        if self.last_sim_time is None:
            self.last_sim_time = self.sim_time
            return

        dt = self.sim_time - self.last_sim_time
        self.last_sim_time = self.sim_time
        if dt <= 0.0 or dt > 0.5:
            return

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.omega
        self.cmd_pub.publish(cmd)

        # Midpoint integration for the estimated global pose.
        theta_mid = self.theta + self.omega * dt / 2.0
        self.x += self.v * math.cos(theta_mid) * dt
        self.y += self.v * math.sin(theta_mid) * dt
        self.theta = (self.theta + self.omega * dt + math.pi) % (2 * math.pi) - math.pi

        pose = Pose2D()
        pose.x, pose.y, pose.theta = self.x, self.y, self.theta
        self.pose_pub.publish(pose)

        if self.sim_time - self.last_log_time >= 1.0:
            self.last_log_time = self.sim_time

            # Ubah kecepatan linear robot menjadi komponen koordinat global.
            vx_global = self.v * math.cos(self.theta)
            vy_global = self.v * math.sin(self.theta)

            self.get_logger().info(
                f"Global Position: X={self.x:.3f} m, Y={self.y:.3f} m, "
                f"Theta={math.degrees(self.theta):.1f} deg | "
                f"Global Velocity: Vx={vx_global:.3f} m/s, "
                f"Vy={vy_global:.3f} m/s | "
                f"Angular Velocity: Omega={self.omega:.3f} rad/s"
            )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    print("\n=== GLOBAL COORDINATE: GERAK MELINGKAR SEPERTI JANGKA ===")
    print("VR = kecepatan linear roda kanan (m/s)")
    print("VL = kecepatan linear roda kiri (m/s)")
    print("VR != VL menghasilkan gerak melingkar/melengkung.")
    print("VR = -VL membuat robot berputar di tempat.\n")
    vr = read_float("Masukkan VR (roda kanan, m/s)")
    vl = read_float("Masukkan VL (roda kiri, m/s)")
    track = read_float("Jarak antar-roda (m)")
    if track <= 0:
        print("Jarak antar-roda harus lebih besar dari 0.")
        return

    rclpy.init(args=args)
    node = GlobalCoordinateCompass(vr, vl, track)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
