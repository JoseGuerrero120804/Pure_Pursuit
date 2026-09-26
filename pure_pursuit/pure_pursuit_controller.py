#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pure_pursuit.geometry import yaw_from_quaternion
from pure_pursuit.path_utils import classify_segments

# Parameters that can be changed live (e.g. `ros2 param set`) without restarting
# the node, so look-ahead tuning (k) and speed can be retuned mid-run.
_RECONFIGURABLE_PARAMS = (
    'wheelbase',
    'lookahead_gain',
    'min_lookahead_distance',
    'linear_velocity',
    'linear_velocity_curve',
    'curve_curvature_threshold',
    'goal_tolerance',
    'max_angular_velocity',
)


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.declare_parameter('path_file', os.path.expanduser('~/waypoints.csv'))
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheelbase', 2.68)
        self.declare_parameter('lookahead_gain', 1.0)
        self.declare_parameter('min_lookahead_distance', 0.5)
        self.declare_parameter('linear_velocity', 1.0)
        self.declare_parameter('linear_velocity_curve', 0.5)
        self.declare_parameter('curve_curvature_threshold', 0.1)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('search_window', 50)

        path_file = self.get_parameter('path_file').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        for name in _RECONFIGURABLE_PARAMS:
            setattr(self, name, self.get_parameter(name).value)
        control_rate = self.get_parameter('control_rate').value
        self.search_window = self.get_parameter('search_window').value

        self.path = self._load_path(path_file)
        self._segment_is_curve = classify_segments(self.path, self.curve_curvature_threshold)
        self._closest_idx = 0
        self._pose = None
        self._goal_reached = False

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(
            f'Loaded {len(self.path)} waypoints from {path_file}, '
            f'L={self.wheelbase} m, v_straight={self.linear_velocity} m/s, '
            f'v_curve={self.linear_velocity_curve} m/s'
        )

    def _on_parameter_update(self, params):
        for p in params:
            if p.name in _RECONFIGURABLE_PARAMS:
                setattr(self, p.name, p.value)
                if p.name == 'curve_curvature_threshold':
                    self._segment_is_curve = classify_segments(self.path, p.value)
        return SetParametersResult(successful=True)

    def _load_path(self, path_file):
        path = []
        with open(path_file, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                path.append((float(row['x']), float(row['y'])))
        if len(path) < 2:
            raise RuntimeError(f'Path file {path_file} must contain at least 2 waypoints')
        return path

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._pose = (x, y, yaw)

    def _advance_closest_index(self, x, y):
        # Only search a bounded window ahead of the last known index, rather than the
        # whole remaining path. An unbounded search can jump far ahead to a point that
        # is merely spatially close (e.g. a path that loops back near its own start),
        # which would make the controller think it had already reached the goal.
        search_end = min(self._closest_idx + self.search_window, len(self.path))
        best_idx = self._closest_idx
        best_dist = math.hypot(self.path[best_idx][0] - x, self.path[best_idx][1] - y)
        for i in range(self._closest_idx + 1, search_end):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_dist:
                best_dist = d
                best_idx = i
        self._closest_idx = best_idx

    def _find_lookahead_point(self, x, y, lookahead_distance):
        self._advance_closest_index(x, y)
        for i in range(self._closest_idx, len(self.path)):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d >= lookahead_distance:
                return self.path[i], i
        return self.path[-1], len(self.path) - 1

    def _target_speed(self, path_idx):
        """Commanded forward speed v_x: reduced through curves, nominal on straights.
        This is what makes the look-ahead distance P = k * v_x actually vary along
        the path, instead of being a fixed constant."""
        seg_idx = min(path_idx, len(self._segment_is_curve) - 1)
        return self.linear_velocity_curve if self._segment_is_curve[seg_idx] else self.linear_velocity

    def _control_loop(self):
        if self._pose is None or self._goal_reached:
            return

        x, y, yaw = self._pose
        v_x = self._target_speed(self._closest_idx)

        # Dynamic look-ahead distance: P = k * v_x
        P = max(self.lookahead_gain * v_x, self.min_lookahead_distance)

        target, target_idx = self._find_lookahead_point(x, y, P)

        last_idx = len(self.path) - 1
        if target_idx == last_idx:
            gx = self.path[-1][0] - x
            gy = self.path[-1][1] - y
            dist_to_goal = math.hypot(gx, gy)
            goal_local_x = math.cos(-yaw) * gx - math.sin(-yaw) * gy
            # Stop on progress, not only distance: within tolerance, nearest to the final
            # waypoint, or already past it (goal behind the car). Otherwise a car that
            # misses the tolerance circle keeps circling the goal at its minimum radius.
            if (dist_to_goal < self.goal_tolerance
                    or self._closest_idx == last_idx
                    or goal_local_x < 0.0):
                self._goal_reached = True
                self._cmd_pub.publish(Twist())
                self.get_logger().info('Goal reached, stopping.')
                return

        # Heading error psi to the look-ahead point, in the vehicle frame.
        dx = target[0] - x
        dy = target[1] - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        psi = math.atan2(local_y, local_x)

        # Pure pursuit steering angle.
        delta = math.atan2(2.0 * self.wheelbase * math.sin(psi), P)

        # Kinematic bicycle model: convert steering angle to yaw rate for /cmd_vel.
        theta_dot = (v_x / self.wheelbase) * math.tan(delta)
        theta_dot = max(min(theta_dot, self.max_angular_velocity), -self.max_angular_velocity)

        cmd = Twist()
        cmd.linear.x = v_x
        cmd.angular.z = theta_dot
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
