#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

import time

class FreeMarker3DoF(Node):
    def __init__(self):
        super().__init__('free_marker_3dof')

        # Params
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'target_pose')
        self.declare_parameter('marker_name', 'free_goal')
        self.declare_parameter('scale', 0.2)
        self.declare_parameter('initial_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('zero_orientation_on_publish', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.marker_name = self.get_parameter('marker_name').value
        self.scale = float(self.get_parameter('scale').value)
        ip = list(self.get_parameter('initial_xyz').value)
        self.zero_orientation = bool(self.get_parameter('zero_orientation_on_publish').value)

        self.pub_pose = self.create_publisher(PoseStamped, self.topic, 10)
        self.server = InteractiveMarkerServer(self, 'free_marker_server')

        self.last_click = time.time()

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = self.marker_name
        int_marker.description = 'Drag (XY)'
        int_marker.scale = self.scale
        int_marker.pose.position.x = ip[0]
        int_marker.pose.position.y = ip[1]
        int_marker.pose.position.z = ip[2]
        int_marker.pose.orientation.w = 1.0  # identity; no rotation controls

        # Visible center marker
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = self.scale * 0.5
        sphere.scale.y = self.scale * 0.5
        sphere.scale.z = self.scale * 0.5
        sphere.color.r = 0.1
        sphere.color.g = 0.3
        sphere.color.b = 0.9
        sphere.color.a = 0.9

        center = InteractiveMarkerControl()
        center.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        center.orientation.w = 1.0
        center.orientation.x = 0.0
        center.orientation.y = 1.0  # aligns the control so movement is in XY
        center.orientation.z = 0.0
        center.always_visible = True
        center.markers.append(sphere)
        int_marker.controls.append(center)

        # XY plane control (plane normal = Z axis)
        xy = InteractiveMarkerControl()
        xy.name = 'move_xy'
        xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        xy.orientation.w = 1.0
        xy.orientation.x = 0.0
        xy.orientation.y = 1.0  # aligns the control so movement is in XY
        xy.orientation.z = 0.0
        int_marker.controls.append(xy)

        # Z axis slider
        move_z = InteractiveMarkerControl()
        move_z.name = 'move_z'
        move_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_z.orientation.w = 1.0
        move_z.orientation.x = 0.0
        move_z.orientation.y = 0.0
        move_z.orientation.z = 1.0   # axis = Z
        #int_marker.controls.append(move_z)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.feedback_cb)  # optional: limit with feedback_type=
        self.server.applyChanges()

        self.get_logger().info(
            f"3-DoF marker '{self.marker_name}' ready in frame '{self.frame_id}'. "
            f"In RViz: Add InteractiveMarkers display, Update Topic: /free_marker_server/update. "
            f"Publishing PoseStamped on: /{self.topic}"
        )

    def feedback_cb(self, feedback):

        if feedback.event_type in (InteractiveMarkerFeedback.BUTTON_CLICK, InteractiveMarkerFeedback.MOUSE_UP):
            if time.time() - self.last_click < 0.5:
                ps = PoseStamped()
                ps.header.frame_id = feedback.header.frame_id
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose = feedback.pose
                if self.zero_orientation:
                    ps.pose.orientation.x = 0.0
                    ps.pose.orientation.y = 0.0
                    ps.pose.orientation.z = 0.0
                    ps.pose.orientation.w = 1.0
                self.pub_pose.publish(ps)

            self.last_click = time.time()

def main():
    rclpy.init()
    node = FreeMarker3DoF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()