#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFAliases(Node):
    def __init__(self):
        super().__init__("tf_aliases")

        # Map unprefixed -> prefixed frames (identity transforms)
        # parent -> child
        self.declare_parameter("aliases", [
            "lidar_link,reblade_bot/lidar_link",
            "camera_link,reblade_bot/camera_link",
            "base_link,reblade_bot/base_link",
        ])

        aliases = list(self.get_parameter("aliases").value)
        self.br = StaticTransformBroadcaster(self)

        tfs = []
        for item in aliases:
            parent, child = [s.strip() for s in item.split(",")]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            # identity transform
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        self.br.sendTransform(tfs)
        self.get_logger().info(f"Published {len(tfs)} static TF aliases")

def main():
    rclpy.init()
    node = TFAliases()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

