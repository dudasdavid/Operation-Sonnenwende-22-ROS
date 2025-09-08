#!/usr/bin/env python3
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


def rot_y_neg(theta: float):
    """Rotation matrix for Ry(-theta)."""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, 0.0, -s),
            (0.0, 1.0, 0.0),
            (s, 0.0,  c))

def rot_z(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return ((c, -s, 0.0),
            (s,  c, 0.0),
            (0.0, 0.0, 1.0))

def matmul3(A, B):
    return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)) for i in range(3))

def Rzy(yaw: float, tilt: float):
    return matmul3(rot_z(yaw), rot_y_neg(tilt))

def matvec3(R, v):
    return (R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2])

def vec_sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vec_add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def vec_mul(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def dot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def norm(a): return math.sqrt(max(1e-12, dot(a,a)))


class AutoAim(Node):
    def __init__(self):
        super().__init__('auto_aim')

        # --- Parameters ---
        self.declare_parameter('target_pose_topic', 'target_pose')

        # Ballistics (your defaults)
        self.declare_parameter('muzzle_speed', 20.0)            # m/s
        self.declare_parameter('mass', 0.0016)                 # kg
        self.declare_parameter('drag_coefficient', 1.1)        # unitless
        self.declare_parameter('cross_section_area', 3.14e-4)  # m^2
        self.declare_parameter('air_density', 1.225)           # kg/m^3
        self.declare_parameter('gravity', 9.80665)             # m/s^2

        # Integrator / solver tuning
        self.declare_parameter('max_flight_time', 5.0)         # s
        self.declare_parameter('dt', 0.02)                    # s
        self.declare_parameter('tilt_min_deg', -30.0)          # kept for completeness (unused externally)
        self.declare_parameter('tilt_max_deg', 60.0)           # kept for completeness (unused externally)
        self.declare_parameter('tilt_coarse_step_deg', 3.0)    # deg
        self.declare_parameter('tilt_refine_span_deg', 6.0)    # ±deg around best
        self.declare_parameter('tilt_refine_step_deg', 0.5)    # deg
        self.declare_parameter('unreachable_warn_mm', 100.0)   # mm miss to warn

        # NEW: hard angle limits (degrees)
        self.declare_parameter('tilt_limit_deg', 30.0)         # allowed final tilt range [-30, +30]
        self.declare_parameter('pan_limit_deg', 40.0)          # allowed final pan range  [-40, +40]

        self.topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value

        # Pull typed params
        self.muzzle_speed = float(self.get_parameter('muzzle_speed').value)
        self.mass = float(self.get_parameter('mass').value)
        self.Cd = float(self.get_parameter('drag_coefficient').value)
        self.area = float(self.get_parameter('cross_section_area').value)
        self.rho = float(self.get_parameter('air_density').value)
        self.g = float(self.get_parameter('gravity').value)

        self.max_flight_time = float(self.get_parameter('max_flight_time').value)
        self.dt = float(self.get_parameter('dt').value)
        self.tilt_coarse_step_deg = float(self.get_parameter('tilt_coarse_step_deg').value)
        self.tilt_refine_span_deg = float(self.get_parameter('tilt_refine_span_deg').value)
        self.tilt_refine_step_deg = float(self.get_parameter('tilt_refine_step_deg').value)
        self.unreachable_warn_mm = float(self.get_parameter('unreachable_warn_mm').value)

        self.tilt_limit_deg = float(self.get_parameter('tilt_limit_deg').value)
        self.pan_limit_deg = float(self.get_parameter('pan_limit_deg').value)

        # Geometry (meters)
        self.pivot = (0.0, 0.0, 0.2551)                  # turret_base -> pan/tilt pivot
        self.muzzle_local = (0.17135, 0.0, -0.0454)      # in gun frame (+x fwd, +z up)

        # Publishers (degrees!)
        self.pub_pan = self.create_publisher(Float32, 'turret_pan_angle_request', 10)
        self.pub_tilt = self.create_publisher(Float32, 'turret_tilt_angle_request', 10)

        # Subscribe
        self.sub = self.create_subscription(PoseStamped, self.topic, self.cb, 10)
        self.get_logger().info(f"auto_aim listening on '{self.topic}'")

    # ---- Ballistic simulator with quadratic drag ----
    def simulate_miss(self, start_pos, v0_vec, target) -> Tuple[float, float]:
        k = 0.5 * self.rho * self.Cd * self.area / self.mass

        pos = start_pos
        vel = v0_vec
        t = 0.0
        min_d = float('inf')
        t_at_min = 0.0

        dt = max(1e-4, float(self.dt))
        tmax = max(dt, float(self.max_flight_time))

        # semi-implicit Euler
        while t < tmax:
            d = norm(vec_sub(pos, target))
            if d < min_d:
                min_d = d
                t_at_min = t

            vmag = norm(vel)
            a_drag = vec_mul(vel, -k * vmag)
            a = (a_drag[0], a_drag[1], a_drag[2] - self.g)

            vel = vec_add(vel, vec_mul(a, dt))
            pos = vec_add(pos, vec_mul(vel, dt))
            t += dt

            if pos[2] < -1.0:  # fell far below plane
                break

        return min_d, t_at_min

    def solve_angles(self, target):
        # Pan (yaw) from pivot->target XY
        to_t = vec_sub(target, self.pivot)
        xy_norm = math.hypot(to_t[0], to_t[1])
        yaw = 0.0 if xy_norm < 1e-9 else math.atan2(to_t[1], to_t[0])

        v0 = self.muzzle_speed

        def miss_for_tilt(tilt):
            R = Rzy(yaw, tilt)
            muzzle_world = vec_add(self.pivot, matvec3(R, self.muzzle_local))
            dir_world = matvec3(R, (1.0, 0.0, 0.0))  # gun forward
            v0_vec = vec_mul(dir_world, v0)
            miss, _ = self.simulate_miss(muzzle_world, v0_vec, target)
            return miss

        # coarse grid
        best_tilt = 0.0
        best_miss = float('inf')
        tmin = math.radians(-30.0)
        tmax = math.radians(60.0)
        tstep = math.radians(max(0.1, self.tilt_coarse_step_deg))

        tilt = tmin
        while tilt <= tmax + 1e-9:
            miss = miss_for_tilt(tilt)
            if miss < best_miss:
                best_miss, best_tilt = miss, tilt
            tilt += tstep

        # refine around best
        span = math.radians(self.tilt_refine_span_deg)
        fine_step = math.radians(max(0.05, self.tilt_refine_step_deg))
        t0 = best_tilt
        tilt = t0 - span
        while tilt <= t0 + span + 1e-12:
            miss = miss_for_tilt(tilt)
            if miss < best_miss:
                best_miss, best_tilt = miss, tilt
            tilt += fine_step

        return yaw, best_tilt, best_miss

    def _clamp_deg(self, name: str, angle_deg: float, limit_deg: float) -> float:
        clamped = max(-limit_deg, min(limit_deg, angle_deg))
        if abs(clamped - angle_deg) > 1e-6:
            self.get_logger().warn(
                f"{name} angle {angle_deg:.2f}° exceeds limit ±{limit_deg:.1f}°, capping to {clamped:.2f}°"
            )
        return clamped

    # ---- Callback ----
    def cb(self, msg: PoseStamped):
        # Target position is in turret_base frame already
        target = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        yaw, tilt, miss = self.solve_angles(target)
        pan_deg = math.degrees(yaw)
        tilt_deg = math.degrees(tilt) * -1

        # Clamp to configured limits
        pan_deg_cmd = self._clamp_deg("Pan", pan_deg, self.pan_limit_deg)
        tilt_deg_cmd = self._clamp_deg("Tilt", tilt_deg, self.tilt_limit_deg)

        miss_mm = miss * 1000.0
        if miss_mm > self.unreachable_warn_mm:
            self.get_logger().warn(
                f"Target @ ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m | "
                f"Pan: {pan_deg:.2f}° -> cmd {pan_deg_cmd:.2f}°, "
                f"Tilt: {tilt_deg:.2f}° -> cmd {tilt_deg_cmd:.2f}° | "
                f"miss ≈ {miss_mm:.1f} mm (may be unreachable)"
            )
        else:
            self.get_logger().info(
                f"Target @ ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m | "
                f"Pan: {pan_deg:.2f}° -> cmd {pan_deg_cmd:.2f}°, "
                f"Tilt: {tilt_deg:.2f}° -> cmd {tilt_deg_cmd:.2f}° | "
                f"miss ≈ {miss_mm:.1f} mm"
            )

        # Publish **degrees** once per request
        pan_msg = Float32(); pan_msg.data = float(pan_deg_cmd)
        tilt_msg = Float32(); tilt_msg.data = float(tilt_deg_cmd)
        self.pub_pan.publish(pan_msg)
        self.pub_tilt.publish(tilt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoAim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()