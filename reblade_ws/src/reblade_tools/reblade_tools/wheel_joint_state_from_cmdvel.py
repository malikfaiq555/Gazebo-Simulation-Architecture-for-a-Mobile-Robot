#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class WheelJointStateFromCmdVel(Node):
    """
    Publishes /joint_states for left/right wheel joints based on /cmd_vel.
    This is purely for RViz TF (robot_state_publisher) when Gazebo doesn't publish joint states.
    """
    def __init__(self):
        super().__init__("wheel_joint_state_from_cmdvel")

        # Match your URDF params
        self.declare_parameter("wheel_radius", 0.06)
        self.declare_parameter("wheel_separation", 0.22)
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("publish_rate_hz", 30.0)

        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_separation").value)
        self.left_joint = str(self.get_parameter("left_joint_name").value)
        self.right_joint = str(self.get_parameter("right_joint_name").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        
        self.wheel_radius = 0.06
        self.wheel_separation = 0.22

        
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.left_pos = 0.0
        self.right_pos = 0.0

        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Publishing /joint_states from /cmd_vel (R={self.R}, L={self.L})"
        )

    def on_cmd(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Convert cmd_vel to wheel angular velocities (rad/s)
        # wl = (v - w*L/2)/R, wr = (v + w*L/2)/R
        wl = (self.cmd_v - self.cmd_w * self.L * 0.5) / self.R
        wr = (self.cmd_v + self.cmd_w * self.L * 0.5) / self.R

        # Integrate to positions for robot_state_publisher
        self.left_pos += wl * dt
        self.right_pos += wr * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [wl, wr]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = WheelJointStateFromCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

