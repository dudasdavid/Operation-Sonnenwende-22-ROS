#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import quaternion_matrix
from builtin_interfaces.msg import Duration as DurationMsg


class NerfTrajectoryNode(Node):
    def __init__(self):
        super().__init__('nerf_trajectory_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('link_name', 'gun_ee_link')
        self.declare_parameter('pan_joint_name', 'pan_joint')
        self.declare_parameter('tilt_joint_name', 'tilt_joint')

        # Ballistic params (tune these to match your blaster + dart)
        self.declare_parameter('muzzle_speed', 20.0)          # m/s (typical Nerf is ~10–30 m/s depending on mod)
        self.declare_parameter('mass', 0.0015)                # kg (about 1.5 g)
        self.declare_parameter('drag_coefficient', 0.9)       # unitless (high drag for foam)
        self.declare_parameter('cross_section_area', 3.14e-4) # m^2 (~2 cm diameter -> pi*(0.01)^2)
        self.declare_parameter('air_density', 1.225)          # kg/m^3
        self.declare_parameter('gravity', 9.80665)            # m/s^2

        # Integration controls
        self.declare_parameter('dt', 0.02)                    # s
        self.declare_parameter('max_flight_time', 5.0)               # s
        self.declare_parameter('ground_z', 0.0)               # m, ground plane height in world_frame

        # Geometry
        self.declare_parameter('barrel_axis', 'x')            # 'x', 'y', or 'z' in link frame
        self.declare_parameter('muzzle_offset_xyz', [0.0, 0.0, 0.0])  # in link frame, meters

        # Markers
        self.declare_parameter('marker_topic', 'nerf_trajectory')
        self.declare_parameter('marker_ns', 'nerf_ballistics')
        self.declare_parameter('line_width', 0.01)            # m
        self.declare_parameter('point_scale', 0.03)           # m
        self.declare_parameter('marker_lifetime', 0.2)        # seconds
        self.declare_parameter('color_rgba', [1.0, 0.6, 0.0, 1.0])  # orange-ish

        self.last_joint_angle_change = time.time()
        self.trajectory = None

        # Read params
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.link_name = self.get_parameter('link_name').get_parameter_value().string_value
        self.pan_joint_name = self.get_parameter('pan_joint_name').get_parameter_value().string_value
        self.tilt_joint_name = self.get_parameter('tilt_joint_name').get_parameter_value().string_value

        self.muzzle_speed = float(self.get_parameter('muzzle_speed').value)
        self.mass = float(self.get_parameter('mass').value)
        self.Cd = float(self.get_parameter('drag_coefficient').value)
        self.area = float(self.get_parameter('cross_section_area').value)
        self.rho = float(self.get_parameter('air_density').value)
        self.g = float(self.get_parameter('gravity').value)
        self.dt = float(self.get_parameter('dt').value)
        self.max_time = float(self.get_parameter('max_flight_time').value)
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.barrel_axis = str(self.get_parameter('barrel_axis').value).lower()
        self.muzzle_offset = [float(x) for x in self.get_parameter('muzzle_offset_xyz').value]

        self.marker_topic = self.get_parameter('marker_topic').value
        self.marker_ns = self.get_parameter('marker_ns').value
        self.line_width = float(self.get_parameter('line_width').value)
        self.point_scale = float(self.get_parameter('point_scale').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime').value)
        self.color = [float(x) for x in self.get_parameter('color_rgba').value]

        # ---------------- TF setup ----------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=0.5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Publishers/Subscribers ----------------
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)

        # Track pan/tilt to trigger recompute (and avoid spamming)
        self._last_pan: Optional[float] = None
        self._last_tilt: Optional[float] = None
        self._dirty = True  # compute once at startup

        # Timer to (re)publish if dirty (and to follow TF changes smoothly)
        self.timer = self.create_timer(0.2, self.timer_cb)  # 10 Hz

        self.get_logger().info('NerfTrajectoryNode ready.')

    # ---------------- Callbacks ----------------
    def on_joint_state(self, msg: JointState):
        # Find pan/tilt if present; if changed beyond tiny epsilon, mark dirty
        try:
            name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
            pan = name_to_pos.get(self.pan_joint_name, self._last_pan)
            tilt = name_to_pos.get(self.tilt_joint_name, self._last_tilt)

            changed = False
            if pan is not None and (self._last_pan is None or abs(pan - self._last_pan) > 1e-5):
                self._last_pan = pan
                changed = True
                self.last_joint_angle_change = time.time()
            if tilt is not None and (self._last_tilt is None or abs(tilt - self._last_tilt) > 1e-5):
                self._last_tilt = tilt
                changed = True
                self.last_joint_angle_change = time.time()

            if changed:
                self._dirty = True
            else:
                # Even if there is no change due to TF delays let's recalculate the trajectory in the following 2 seconds after no change
                if time.time() - self.last_joint_angle_change < 2:
                    self._dirty = True

        except Exception as e:
            self.get_logger().warn(f'Joint state processing error: {e}')

    def timer_cb(self):
        if self._dirty:
            self.trajectory = self.compute_trajectory()
        if self.trajectory is not None and len(self.trajectory) >= 2:
            self.publish_markers(self.trajectory)
            self._dirty = False

    # ---------------- Core logic ----------------
    def compute_trajectory(self):
        """
        Integrate projectile motion with quadratic drag in world_frame,
        starting from the muzzle pose derived from link_name TF.
        Stops when crossing ground_z downward or exceeding max_time.
        """
        try:
            now = self.get_clock().now() - Duration(seconds=0.5)
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.link_name, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f'Cannot get TF {self.world_frame} <- {self.link_name}: {ex}')
            return None

        # Position of link origin in world frame
        p0 = (tf.transform.translation.x,
              tf.transform.translation.y,
              tf.transform.translation.z)

        # Rotation (link->world) as 4x4
        q = tf.transform.rotation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])  # returns 4x4
        R = T[:3, :3]

        # Muzzle position = link origin + R * muzzle_offset
        o_l = self.muzzle_offset
        o_w = R @ o_l
        start = (p0[0] + o_w[0], p0[1] + o_w[1], p0[2] + o_w[2])

        # Forward axis in link frame
        if self.barrel_axis == 'x':
            f_l = (1.0, 0.0, 0.0)
        elif self.barrel_axis == 'y':
            f_l = (0.0, 1.0, 0.0)
        elif self.barrel_axis == 'z':
            f_l = (0.0, 0.0, 1.0)
        else:
            self.get_logger().warn(f'Unknown barrel_axis "{self.barrel_axis}", defaulting to +X.')
            f_l = (1.0, 0.0, 0.0)

        # Forward axis in world frame
        f_w = R @ f_l
        f_norm = math.sqrt(f_w[0]**2 + f_w[1]**2 + f_w[2]**2) or 1.0
        dir_w = (f_w[0]/f_norm, f_w[1]/f_norm, f_w[2]/f_norm)

        # Initial velocity
        v = [self.muzzle_speed * dir_w[0],
             self.muzzle_speed * dir_w[1],
             self.muzzle_speed * dir_w[2]]

        # Integration
        p = [start[0], start[1], start[2]]
        t = 0.0
        points = [Point(x=p[0], y=p[1], z=p[2])]

        # Helper for drag
        k = 0.5 * self.rho * self.Cd * self.area / self.mass
        g_vec = [0.0, 0.0, -self.g]

        max_steps = max(2, int(self.max_time / self.dt))
        for _ in range(max_steps):
            # Quadratic drag: a_drag = -(k * |v|) * v
            speed = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
            a_drag = [-k * speed * v[0], -k * speed * v[1], -k * speed * v[2]]
            a = [g_vec[0] + a_drag[0],
                 g_vec[1] + a_drag[1],
                 g_vec[2] + a_drag[2]]

            # Semi-implicit Euler: v <- v + a*dt, p <- p + v*dt
            v[0] += a[0] * self.dt
            v[1] += a[1] * self.dt
            v[2] += a[2] * self.dt

            p_prev = p.copy()
            p[0] += v[0] * self.dt
            p[1] += v[1] * self.dt
            p[2] += v[2] * self.dt

            t += self.dt
            points.append(Point(x=p[0], y=p[1], z=p[2]))

            # Stop when we cross the ground plane downward
            if p[2] <= self.ground_z:
                # Linear interpolate last segment to ground_z for a clean end point
                dz = p[2] - p_prev[2]
                if abs(dz) > 1e-6:
                    alpha = (self.ground_z - p_prev[2]) / dz  # in [0,1]
                    xg = p_prev[0] + alpha * (p[0] - p_prev[0])
                    yg = p_prev[1] + alpha * (p[1] - p_prev[1])
                    points[-1] = Point(x=xg, y=yg, z=self.ground_z)
                break

        return points

    # ---------------- Marker publishing ----------------
    def _marker_base(self, marker_id: int, mtype: int) -> Marker:
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = self.marker_ns
        m.id = marker_id
        m.type = mtype
        m.action = Marker.ADD
        m.lifetime = DurationMsg(sec=int(self.marker_lifetime),
                                 nanosec=int((self.marker_lifetime % 1.0) * 1e9))
        m.color.r, m.color.g, m.color.b, m.color.a = self.color
        return m

    def publish_markers(self, points):
        ma = MarkerArray()

        # Line strip
        line = self._marker_base(0, Marker.LINE_STRIP)
        line.scale.x = self.line_width   # only x is used for line width
        line.points = points

        # Sample points (optional, nice in RViz)
        dots = self._marker_base(1, Marker.SPHERE_LIST)
        dots.scale.x = dots.scale.y = dots.scale.z = self.point_scale
        dots.points = points

        ma.markers.append(line)
        ma.markers.append(dots)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = NerfTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()