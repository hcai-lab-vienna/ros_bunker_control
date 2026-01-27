import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped

class FollowPathActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

    def send_goal(self):
        goal_msg = FollowPath.Goal()
        goal_msg.path.header.frame_id = "odom"
        next_waypoint = PoseStamped()
        next_waypoint.header.frame_id = "base_link"
        next_waypoint.pose.position.x = 1.
        next_waypoint.pose.position.y = 0.5
        goal_msg.path.poses.append(next_waypoint)

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    try:
        with rclpy.init(args=args):

            action_client = FollowPathActionClient()

            future = action_client.send_goal()

            rclpy.spin_until_future_complete(action_client, future)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
