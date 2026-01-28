#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class ExecutePlannedPath(Node):
    def __init__(self):
        super().__init__('follow_planned_path')
        self.declare_parameter('goal_topic', '/controller_goal')
        self.declare_parameter('goal_active_srv', '/is_goal_active')
        self.declare_parameter('odom_frame_id', 'odom')

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.goal_active_srv = str(self.get_parameter('goal_active_srv').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.is_goal_active = self.create_client(Trigger, self.goal_active_srv)

        self.path = Path()
        for i in range(2):
            next_waypoint = PoseStamped()
            next_waypoint.header.frame_id = "base_link"
            next_waypoint.pose.position.x = 1.
            next_waypoint.pose.position.y = 0.5
            self.path.poses.append(next_waypoint)

        self.get_logger().info(f'Node Ready. Setup finished')


    def execute_plan(self):
        for wpt_idx, waypoint in enumerate(self.path.poses):
            self.goal_pub.publish(waypoint)
            self.get_logger().info(f'Navigating to the next waypoint: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f} in {waypoint.header.frame_id})')

            while rclpy.ok():
                future = self.is_goal_active.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(
                    self, future)
                is_goal_active = future.result()
                if not is_goal_active.success:
                    break

                sleep(0.1)

            self.get_logger().info(f'Reached waypoint {wpt_idx+1}.')

        self.get_logger().info(f'Done with plan execution.')


def main(args=None):
    rclpy.init(args=args)
    node = ExecutePlannedPath()

    input("Press ENTER to continue")
    try:
        node.execute_plan()
    except KeyboardInterrupt:
        pass

    stop_goal_pose = PoseStamped()
    stop_goal_pose.header.frame_id = "base_link"
    node.goal_pub.publish(stop_goal_pose)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
