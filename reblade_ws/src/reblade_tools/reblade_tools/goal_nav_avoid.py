#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_min(vals):
    m = float("inf")
    for v in vals:
        if math.isfinite(v) and v > 0.0:
            m = min(m, v)
    return m


class GoalNavAvoid(Node):
    def __init__(self):
        super().__init__("goal_nav_avoid")

        # --- Navigation tunables ---
        self.declare_parameter("forward_speed", 0.20)     # reduced a bit (less tunneling)
        self.declare_parameter("max_turn", 0.9)           # slightly lower -> less aggressive
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("yaw_gain", 1.6)

        # --- Obstacle tunables ---
        self.declare_parameter("enter_avoid_dist", 0.75)  # start avoiding
        self.declare_parameter("exit_avoid_dist", 0.95)   # stop avoiding (hysteresis)
        self.declare_parameter("front_arc_deg", 55)
        self.declare_parameter("avoid_forward_speed", 0.10)
        self.declare_parameter("avoid_turn_gain", 2.5)    # proportional turn strength

        # --- Output smoothing ---
        self.declare_parameter("cmd_smoothing_alpha", 0.25)  # 0..1 (higher = more responsive)

        # Waypoints (x1,y1,x2,y2,...)
        self.declare_parameter("waypoints", [2.0, 0.0,  4.0, 0.0,  6.0, 0.0,  0.0, 0.0])

        self.v_fwd = float(self.get_parameter("forward_speed").value)
        self.max_turn = float(self.get_parameter("max_turn").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)

        self.enter_dist = float(self.get_parameter("enter_avoid_dist").value)
        self.exit_dist = float(self.get_parameter("exit_avoid_dist").value)
        self.front_arc_deg = float(self.get_parameter("front_arc_deg").value)
        self.avoid_v = float(self.get_parameter("avoid_forward_speed").value)
        self.avoid_turn_gain = float(self.get_parameter("avoid_turn_gain").value)

        self.alpha = float(self.get_parameter("cmd_smoothing_alpha").value)
        self.alpha = max(0.0, min(1.0, self.alpha))

        wp_flat = list(self.get_parameter("waypoints").value)
        self.waypoints = [(wp_flat[i], wp_flat[i + 1]) for i in range(0, len(wp_flat), 2)]
        self.wp_idx = 0

        self.pose = None
        self.scan = None

        # Avoidance state
        self.avoiding = False
        self.avoid_dir = 1.0  # +1 left, -1 right

        # For smoothing
        self.prev_v = 0.0
        self.prev_w = 0.0

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.step)

        self.get_logger().info(f"goal_nav_avoid: {len(self.waypoints)} waypoints loaded")

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def scan_cb(self, msg):
        self.scan = msg

    def compute_front_dists(self):
        """Return (d_front, d_left, d_right) for the front arc."""
        if self.scan is None or len(self.scan.ranges) == 0:
            return float("inf"), float("inf"), float("inf")

        scan = self.scan
        n = len(scan.ranges)

        def angle_to_index(angle_rad):
            i = int((angle_rad - scan.angle_min) / scan.angle_increment)
            return max(0, min(n - 1, i))

        front = math.radians(self.front_arc_deg)
        i0 = angle_to_index(-front)
        i1 = angle_to_index(+front)
        mid = (i0 + i1) // 2

        front_slice = scan.ranges[i0:i1 + 1]
        left_slice = scan.ranges[mid:i1 + 1]
        right_slice = scan.ranges[i0:mid + 1]

        d_front = finite_min(front_slice)
        d_left = finite_min(left_slice)
        d_right = finite_min(right_slice)

        return d_front, d_left, d_right

    def obstacle_override(self):
        d_front, d_left, d_right = self.compute_front_dists()

        # Enter avoidance
        if (not self.avoiding) and (d_front < self.enter_dist):
            self.avoiding = True
            self.avoid_dir = 1.0 if d_left > d_right else -1.0

        # If avoiding, stay until clear (hysteresis)
        if self.avoiding:
            if d_front > self.exit_dist:
                self.avoiding = False
                return None

            # Proportional turn: closer -> turn more
            closeness = max(0.0, self.enter_dist - d_front)
            turn = min(self.max_turn, self.avoid_turn_gain * closeness)

            cmd = Twist()
            cmd.linear.x = self.avoid_v
            cmd.angular.z = self.avoid_dir * turn
            return cmd

        return None

    def smooth(self, cmd: Twist) -> Twist:
        # Low-pass filter for smoother motion
        out = Twist()
        out.linear.x = self.alpha * cmd.linear.x + (1.0 - self.alpha) * self.prev_v
        out.angular.z = self.alpha * cmd.angular.z + (1.0 - self.alpha) * self.prev_w
        self.prev_v = out.linear.x
        self.prev_w = out.angular.z
        return out

    def step(self):
        if self.pose is None or not self.waypoints:
            return

        # 1) Obstacle override
        avoid_cmd = self.obstacle_override()
        if avoid_cmd is not None:
            self.pub_cmd.publish(self.smooth(avoid_cmd))
            return

        # 2) Goal navigation
        gx, gy = self.waypoints[self.wp_idx]
        x = self.pose.position.x
        y = self.pose.position.y
        yaw = yaw_from_quat(self.pose.orientation)

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.get_logger().info(f"Reached waypoint -> #{self.wp_idx}: {self.waypoints[self.wp_idx]}")
            return

        target_yaw = math.atan2(dy, dx)
        err = wrap(target_yaw - yaw)

        cmd = Twist()
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, self.yaw_gain * err))

        # reduce speed when turning
        turn_mag = abs(cmd.angular.z)
        cmd.linear.x = self.v_fwd * max(0.05, 1.0 - (turn_mag / self.max_turn) * 0.75)

        self.pub_cmd.publish(self.smooth(cmd))


def main():
    rclpy.init()
    node = GoalNavAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

