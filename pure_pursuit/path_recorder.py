#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from pure_pursuit.geometry import yaw_from_quaternion


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_file', os.path.expanduser('~/waypoints.csv'))
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('max_jump_distance', 1.0)

        odom_topic = self.get_parameter('odom_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_jump_distance = self.get_parameter('max_jump_distance').value

        self._points = []
        self._last_point = None
        self._recording = True

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(
            f'Recording odometry from {odom_topic} (min_distance={self.min_distance} m). '
            f'Press Ctrl+C to stop and write {self.output_file}'
        )

    def _odom_cb(self, msg):
        if not self._recording:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_point is not None:
            dx = x - self._last_point[0]
            dy = y - self._last_point[1]
            jump = math.hypot(dx, dy)
            if jump > self.max_jump_distance:
                self._recording = False
                self.get_logger().warn(
                    f'Odometry jumped {jump:.2f} m in one update (likely a simulation '
                    'reset). Stopping recording to avoid corrupting the path; '
                    f'{len(self._points)} waypoints recorded so far were kept.'
                )
                return
            if jump < self.min_distance:
                return

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self._points.append((x, y, yaw, v, w))
        self._last_point = (x, y)

    def save(self):
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])
            writer.writerows(self._points)
        self.get_logger().info(f'Saved {len(self._points)} waypoints to {self.output_file}')


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
