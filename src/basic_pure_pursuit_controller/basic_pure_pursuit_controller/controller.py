#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_srvs.srv import Trigger


def yaw_from_quat(qx, qy, qz, qw) -> float:
    # Extract yaw from quaternion (ENU)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_robot(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    # Transform world delta into robot frame (x forward, y left)
    c = math.cos(yaw)
    s = math.sin(yaw)
    x_r = c * dx + s * dy
    y_r = -s * dx + c * dy
    return x_r, y_r


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('lookahead_distance', 0.5)     # m
        self.declare_parameter('desired_speed', 0.3)          # m/s
        self.declare_parameter('max_angular_speed', 1.2)      # rad/s
        self.declare_parameter('goal_tolerance', 0.1)         # m
        self.declare_parameter('control_rate', 30.0)          # Hz
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/controller_goal')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('cmd_frame_id', 'base_link')
        self.declare_parameter('stop_when_no_goal', True)
        self.declare_parameter('publish_twist_stamped', True)

        self.Ld = float(self.get_parameter('lookahead_distance').value)
        self.v_des = float(self.get_parameter('desired_speed').value)
        self.w_max = float(self.get_parameter('max_angular_speed').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.rate_hz = float(self.get_parameter('control_rate').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.cmd_frame_id = str(self.get_parameter('cmd_frame_id').value)
        self.stop_when_no_goal = bool(self.get_parameter('stop_when_no_goal').value)
        self.publish_twist_stamped = bool(self.get_parameter('publish_twist_stamped').value)

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.goal_active = False
        self.last_odom_time: Optional[rclpy.time.Time] = None
        self.last_odom_now: Optional[rclpy.time.Time] = None
        self.odom_frame_id: Optional[str] = None

        # ROS I/O
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.srv = self.create_service(Trigger, 'is_goal_active', self.goal_active_cb)
        if self.publish_twist_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(
            1.0 / self.rate_hz,
            self.control_step
        )

        self.get_logger().info(f'Pure pursuit (single waypoint) running: Ld={self.Ld} m, v={self.v_des} m/s, w_max={self.w_max} rad/s')
        self.get_logger().info(f'odom: {self.odom_topic}, cmd: {self.cmd_topic}')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.last_odom_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self.odom_frame_id = msg.header.frame_id
        self.robot_x = p.x
        self.robot_y = p.y
        self.robot_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_odom_now = self.get_clock().now()

    def goal_cb(self, msg: PoseStamped):
        if self.odom_frame_id and msg.header.frame_id and msg.header.frame_id != self.odom_frame_id:
            self.get_logger().warning(f'Goal frame "{msg.header.frame_id}" != odom frame "{self.odom_frame_id}".')
            self.get_logger().info(f'Waiting for transform {msg.header.frame_id} to {self.odom_frame_id} ...')
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame_id,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            # Apply the transform to the PoseStamped
            msg = do_transform_pose_stamped(msg, transform)

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_frame_id = msg.header.frame_id
        self.goal_active = True
        self.get_logger().info(f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def goal_active_cb(self, request, response):
        response.success = self.goal_active
        return response

    def control_step(self):
        robot_x = self.robot_x
        robot_y = self.robot_y
        robot_yaw = self.robot_yaw

        if not self.goal_active:
            self.publish_cmd(0.0, 0.0)
            return

        # Compute vector to goal
        dx = self.goal_x - robot_x
        dy = self.goal_y - robot_y
        dist = math.hypot(dx, dy)

        # Goal reached
        if dist < self.goal_tol:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal reached.')
            self.goal_active = False
            return

        # Choose lookahead point along the line to the goal
        if dist > self.Ld:
            scale = self.Ld / dist
            target_x = robot_x + scale * dx
            target_y = robot_y + scale * dy
        else:
            target_x = self.goal_x
            target_y = self.goal_y

        # Transform to robot frame
        tx_r, ty_r = world_to_robot(target_x - robot_x, target_y - robot_y, robot_yaw)
        Ld_eff = max(math.hypot(tx_r, ty_r), 1e-6)

        # Pure pursuit curvature and commands
        kappa = 2.0 * ty_r / (Ld_eff * Ld_eff)
        v_cap = min(self.v_des, self.w_max / max(abs(kappa), 1e-6))

        # Slow down near goal
        slow_factor = max(min(dist / max(self.Ld, 1e-3), 1.0), 0.2)
        v_cmd = min(v_cap, self.v_des * slow_factor)
        w_cmd = max(min(v_cmd * kappa, self.w_max), -self.w_max)

        self.publish_cmd(v_cmd, w_cmd)

    def publish_cmd(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        if self.publish_twist_stamped:
            self.cmd_pub.publish(msg)
        else:
            self.cmd_pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    control_node = PurePursuitController()

    try:
        rclpy.spin(control_node)
    finally:
        control_node.publish_cmd(0.0, 0.0)
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
