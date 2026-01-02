#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameFix(Node):
    def __init__(self):
        super().__init__("scan_frame_fix")
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, 10)
        self.pub = self.create_publisher(LaserScan, "/scan_fixed", 10)
        self.get_logger().info("Republishing /scan -> /scan_fixed with stripped frame_id")

    def cb(self, msg: LaserScan):
        out = LaserScan()
        out = msg  # shallow copy OK for this use
        out.header.frame_id = msg.header.frame_id.split("/")[-1]
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ScanFrameFix()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

