#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from std_msgs.msg import UInt8
from turret_interfaces.msg import GunFeedback
import time

class TurretInteractiveMarker(Node):
    def __init__(self):
        super().__init__('turret_interactive_marker')

        # Publisher for shot request
        self.shot_pub = self.create_publisher(UInt8, '/turret_shoot_request', 10)
        self.feedback_sub = self.create_subscription(
            GunFeedback,
            '/turret_gun_feedback',
            self.gun_feedback_cb,
            10
        )

        self.last_click = time.time()
        self.publish_time = time.time()

        # State
        self.current_ammo = 0
        self.current_voltage = 0.0
        self.magazine = False
        self.distance = 0
        self.marker_name = "turret_sphere"
        self.marker_scale = 0.2  # sphere diameter (m)

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, "turret_marker")

        # Build and insert initial marker
        self.insert_or_update_marker()

        self.get_logger().info("Turret interactive marker initialized (double-click to fire).")

    # ----- Marker building / updating -----
    def _build_marker(self) -> InteractiveMarker:
        im = InteractiveMarker()
        im.header.frame_id = "gun_ee_link"
        im.name = self.marker_name
        im.description = f"Ammo: {self.current_ammo}\nVoltage: {self.current_voltage:.2f}V\nMagazine: {self.magazine}\nDistance: {self.distance}mm"
        im.scale = 0.3

        # Sphere visualization
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.1
        sphere.scale.y = 0.1
        sphere.scale.z = 0.1
        sphere.color.r = 0.80
        sphere.color.g = 0.10
        sphere.color.b = 0.10
        sphere.color.a = 0.50

        # Floating text displaying ammo (view-facing), above the sphere
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.scale.z = self.marker_scale * 0.8  # text height
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.pose.position.z = self.marker_scale * 0.9
        text.text = f"Ammo: {self.current_ammo}"

        # Control: BUTTON -> not movable, just clickable/double-clickable
        ctrl = InteractiveMarkerControl()
        ctrl.always_visible = True
        ctrl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctrl.markers.append(sphere)
        #ctrl.markers.append(text)

        im.controls.append(ctrl)
        return im

    def insert_or_update_marker(self):
        int_marker = self._build_marker()
        # Insert with feedback callback; inserting with the same name updates it
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback) 
        self.server.applyChanges()

    def gun_feedback_cb(self, msg: GunFeedback):
        # Update cached ammo and refresh marker visuals + description
        new_ammo = int(msg.ammo)
        new_voltage = float(msg.voltage)
        new_magazine = bool(msg.magazine)
        new_distance = int(msg.distance_mm)
        if new_ammo != self.current_ammo:
            self.current_ammo = new_ammo
            self.current_voltage = new_voltage
            self.magazine = new_magazine
            self.distance = new_distance
            self.insert_or_update_marker()

    def process_feedback(self, feedback):
        # Double click interaction
        if feedback.event_type in (InteractiveMarkerFeedback.BUTTON_CLICK, InteractiveMarkerFeedback.MOUSE_UP):
            # detect double click and avoid multiple publishing on the 3rd, 4th, etc click
            if time.time() - self.last_click < 0.5 and time.time() - self.publish_time > 2.0:
                msg = UInt8()
                msg.data = 1
                self.shot_pub.publish(msg)
                self.get_logger().info("Double click detected -> Shot request published.")
                self.publish_time = time.time()
            self.last_click = time.time()

    def destroy_node(self):
        # Clear server on shutdown
        try:
            self.server.clear()
            self.server.applyChanges()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurretInteractiveMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.server.clear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()