#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math

def quaternion_from_yaw(yaw):
    """Convert yaw (in radians) to quaternion (x, y, z, w)"""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

class Nav2WaypointNavigator(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = self.define_waypoints()
        self.goal_index = 0
        self.send_next_goal()

    def define_waypoints(self):
        waypoints = []

        base_x = 11.195
        base_y = -2.63
        distance = 0.6
        num_goals = 8  # total number of waypoints

        for i in range(num_goals):
            x = base_x + i * distance
            y = base_y
            yaw = 0.0  # facing straight

            q = quaternion_from_yaw(yaw)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            waypoints.append(goal)

        return waypoints

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("✅ All waypoints reached.")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goals[self.goal_index]

        self.get_logger().info(f"🚀 Sending goal {self.goal_index + 1}: x={goal_msg.pose.pose.position.x:.2f}, y={goal_msg.pose.pose.position.y:.2f}")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected.')
            return

        self.get_logger().info('✅ Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🏁 Reached goal {self.goal_index + 1}. Pausing for 2 seconds...')
        time.sleep(2)  # Pause at the waypoint
        self.goal_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
