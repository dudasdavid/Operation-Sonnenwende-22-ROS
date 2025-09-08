#!/usr/bin/env python3
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def rot_y_neg(theta: float):
    """Rotation matrix for Ry(-theta)."""
    c, s = math.cos(theta), math.sin(theta)
    # Ry(-θ) = [[ cosθ, 0, -sinθ],
    #           [   0 , 1,    0  ],
    #           [ sinθ, 0,  cosθ]]
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
        # Ballistic params (tune these to match your blaster + dart)
        self.declare_parameter('muzzle_speed', 20.0)          # m/s (typical Nerf is ~10–30 m/s depending on mod)
        self.declare_parameter('mass', 0.0015)                # kg (about 1.5 g)
        self.declare_parameter('drag_coefficient', 0.9)       # unitless (high drag for foam)
        self.declare_parameter('cross_section_area', 3.14e-4) # m^2 (~2 cm diameter -> pi*(0.01)^2)
        self.declare_parameter('air_density', 1.225)          # kg/m^3
        self.declare_parameter('gravity', 9.80665)            # m/s^2

        self.topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        self.muzzle_speed = float(self.get_parameter('muzzle_speed').value)
        self.mass = float(self.get_parameter('mass').value)
        self.Cd = float(self.get_parameter('drag_coefficient').value)
        self.area = float(self.get_parameter('cross_section_area').value)
        self.rho = float(self.get_parameter('air_density').value)
        self.g = float(self.get_parameter('gravity').value)

        # Geometry (meters)
        self.pivot = (0.0, 0.0, 0.2551)                  # turret_base -> gun center (pan/tilt pivot)
        self.muzzle_local = (0.17135, 0.0, -0.0454)      # in gun frame (+x forward, +z up)

        # Subscribe
        self.sub = self.create_subscription(PoseStamped, self.topic, self.cb, 10)
        self.get_logger().info(f"auto_aim listening on '{self.topic}'")

    # ---- Ballistic simulator with drag ----
    def simulate_miss(self, start_pos, v0_vec, target, max_t=3.0, dt=0.002) -> Tuple[float, float]:
        """Simulate trajectory and return (min_distance_to_target, time_at_min)."""
        rho = self.rho
        Cd = self.Cd
        A  = self.area
        m  = self.mass
        g  = self.g

        k = 0.5 * rho * Cd * A / m
        pos = start_pos
        vel = v0_vec
        t = 0.0
        min_d = float('inf')
        t_at_min = 0.0

        # simple (semi-implicit) Euler
        while t < max_t:
            # distance to target
            d = norm(vec_sub(pos, target))
            if d < min_d:
                min_d = d
                t_at_min = t

            vmag = norm(vel)
            # gravity points -Z
            a_drag = vec_mul(vel, -k * vmag)
            a = (a_drag[0], a_drag[1], a_drag[2] - g)

            # integrate
            vel = vec_add(vel, vec_mul(a, dt))
            pos = vec_add(pos, vec_mul(vel, dt))
            t += dt

            # quick exit: if we flew well past target in XY and are descending deep
            if pos[2] < -1.0:
                break

        return min_d, t_at_min

    def solve_angles(self, target):
        # Compute pan (yaw) from pivot->target XY
        to_t = vec_sub(target, self.pivot)
        xy_norm = math.hypot(to_t[0], to_t[1])
        yaw = 0.0 if xy_norm < 1e-6 else math.atan2(to_t[1], to_t[0])

        # 1-D search over tilt (pitch), since yaw is set
        # Build rotation for each tilt, rotate muzzle & direction, simulate, choose best
        v0 = self.muzzle_speed

        def miss_for_tilt(tilt):
            R = Rzy(yaw, tilt)
            muzzle_world = vec_add(self.pivot, matvec3(R, self.muzzle_local))
            dir_world = matvec3(R, (1.0, 0.0, 0.0))  # gun forward
            v0_vec = vec_mul(dir_world, v0)
            miss, _ = self.simulate_miss(muzzle_world, v0_vec, target)
            return miss

        # coarse grid, then refine
        best_tilt = None
        best_miss = float('inf')

        # coarse: -30°..+60°
        for tilt_deg in range(-30, 61, 3):
            tilt = math.radians(tilt_deg)
            miss = miss_for_tilt(tilt)
            if miss < best_miss:
                best_miss, best_tilt = miss, tilt

        # refine around best ±5°
        span = math.radians(6.0)
        step = math.radians(0.5)
        t0 = best_tilt
        tilt = t0 - span
        while tilt <= t0 + span:
            miss = miss_for_tilt(tilt)
            if miss < best_miss:
                best_miss, best_tilt = miss, tilt
            tilt += step

        return yaw, best_tilt, best_miss

    # ---- Callback ----
    def cb(self, msg: PoseStamped):
        # Target position is in turret_base frame already
        target = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        yaw, tilt, miss = self.solve_angles(target)
        yaw_deg = math.degrees(yaw)
        tilt_deg = math.degrees(tilt)

        self.get_logger().info(
            f"Target @ ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m | "
            f"Pan (yaw): {yaw_deg:.2f}°, Tilt (pitch): {tilt_deg:.2f}° | "
            f"sim miss ≈ {miss*1000:.1f} mm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoAim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()