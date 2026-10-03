#!/usr/bin/env python3
"""
odom_to_baselink_tf.py

Subscribes to your teammate's odometry data and publishes the
odom -> base_link transform on /tf, so it's usable by SLAM Toolbox,
Nav2, and the base_link -> laser_link transform from the lidar teammate.

======================================================================
 BEFORE RUNNING: check your teammate's "odom" file and fill in the
 THREE things marked "CHANGE THIS" below to match exactly what they
 publish. Topic names and frame names must match EXACTLY (letter for
 letter) or this will silently receive nothing.
======================================================================
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry              # CHANGE THIS if your teammate publishes a
                                                 # different message type (e.g. geometry_msgs/PoseStamped,
                                                 # or a custom msg)
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ---------------- CHANGE THESE THREE THINGS ----------------
INPUT_TOPIC = '/odom_raw'      # CHANGE THIS: the exact topic name your teammate's odom file publishes to
PARENT_FRAME = 'odom'          # CHANGE THIS if your teammate uses a different frame name (check their file)
CHILD_FRAME = 'base_link'      # CHANGE THIS if your teammate/laser file expects a different name
                                # (e.g. 'base_footprint' — must match what the laser file expects too)
# -------------------------------------------------------------


class OdomToBaseLinkTF(Node):
    def __init__(self):
        super().__init__('odom_to_baselink_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to teammate's odometry data
        self.subscription = self.create_subscription(
            Odometry,
            INPUT_TOPIC,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Listening on "{INPUT_TOPIC}", publishing tf "{PARENT_FRAME}" -> "{CHILD_FRAME}"'
        )

    def odom_callback(self, msg: Odometry):
        # Pull position + orientation straight out of the incoming message.
        # (Odometry messages already carry a ready-to-use quaternion, so no
        # manual math is needed here — just repackage it as a TransformStamped.)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id = CHILD_FRAME

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToBaseLinkTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
