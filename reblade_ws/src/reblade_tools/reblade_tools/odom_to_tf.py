#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate_hz', 30.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.br = TransformBroadcaster(self)

        self.latest_pose = None  # geometry_msgs/Pose
        self.latest_parent = "odom"
        self.latest_child = "base_link"

        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 50)

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.publish_tf)

        self.get_logger().info(
            f"odom_to_tf: listening {self.odom_topic}, publishing TF at {self.publish_rate_hz} Hz (stable)"
        )

    def odom_cb(self, msg: Odometry):
        if not msg.header.frame_id or not msg.child_frame_id:
            return

        # Strip possible "reblade_bot/" prefix
        self.latest_parent = msg.header.frame_id.split('/')[-1]   # odom
        self.latest_child  = msg.child_frame_id.split('/')[-1]    # base_link
        self.latest_pose = msg.pose.pose

    def publish_tf(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now().to_msg()  # sim time if use_sim_time=True

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.latest_parent
        t.child_frame_id = self.latest_child

        t.transform.translation.x = self.latest_pose.position.x
        t.transform.translation.y = self.latest_pose.position.y
        t.transform.translation.z = self.latest_pose.position.z
        t.transform.rotation = self.latest_pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

