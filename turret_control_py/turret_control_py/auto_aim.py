#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class AutoAim(Node):
    def __init__(self):
        super().__init__('auto_aim')

        # Declare parameter for topic name
        self.declare_parameter('target_pose_topic', 'target_pose')
        target_pose_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.target_pose_callback,
            10
        )

        self.get_logger().info(f"auto_aim node started, listening on '{target_pose_topic}'")

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"New target pose received:\n"
            f"  Position -> x: {msg.pose.position.x:.2f}, "
            f"y: {msg.pose.position.y:.2f}, "
            f"z: {msg.pose.position.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AutoAim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()