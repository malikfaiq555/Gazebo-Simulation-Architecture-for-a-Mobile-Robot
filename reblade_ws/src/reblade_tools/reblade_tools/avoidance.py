#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def finite_min(vals):
    m = float("inf")
    for v in vals:
        if math.isfinite(v) and v > 0.0:
            m = min(m, v)
    return m


class ReactiveAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_avoidance')

        # Tunables
        self.forward_speed = 0.25     # m/s
        self.turn_speed = 0.8         # rad/s
        self.min_dist = 0.6           # m (trigger distance)
        self.front_arc_deg = 50       # +/- degrees for "front"

        self.latest_scan = None

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("reactive_avoidance started")

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def control_loop(self):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        n = len(scan.ranges)
        if n == 0:
            return

        # Convert degrees to indices around angle_min..angle_max
        def angle_to_index(angle_rad):
            i = int((angle_rad - scan.angle_min) / scan.angle_increment)
            return max(0, min(n - 1, i))

        front = math.radians(self.front_arc_deg)
        i0 = angle_to_index(-front)
        i1 = angle_to_index(+front)

        # Sectors: left/front/right
        mid = (i0 + i1) // 2
        left_slice = scan.ranges[mid:i1+1]
        right_slice = scan.ranges[i0:mid+1]
        front_slice = scan.ranges[i0:i1+1]

        d_front = finite_min(front_slice)
        d_left = finite_min(left_slice)
        d_right = finite_min(right_slice)

        cmd = Twist()

        # If obstacle in front arc within min_dist -> turn away
        if d_front < self.min_dist:
            cmd.linear.x = 0.0
            # Turn toward the more open side
            if d_left > d_right:
                cmd.angular.z = +self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ReactiveAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

