#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
# from tf2_ros.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

import rclpy
from rclpy.node import Node

def quaternion_from_yaw(yaw):
    """Convert yaw (in radians) to quaternion (x, y, z, w)"""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)
def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy   # w
    ]
    return q
class Nav2WaypointNavigator(Node):
    def __init__(self):
        super().__init__('nav2_waypoint_navigator')
        
        # Flags and variables
        self.received_odom = False
        self.base_x = None
        self.base_y = None
        self.goals = []
        self.goal_index = 0

        # Subscribe to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Set up the action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def odom_callback(self, msg):
        if self.received_odom:
            return  # Only process first odom message

        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        self.received_odom = True

        self.get_logger().info(f"✅ Received /odom position: x={self.base_x}, y={self.base_y}")

        # Now that odom is received, generate waypoints and start navigation
        self.goals = self.define_waypoints()
        self.get_logger().info(f"🚧 Generated {len(self.goals)} waypoints. Beginning navigation...")
        self.send_next_goal()


    def define_waypoints(self):
        waypoints = []

        distance = 0.6
        num_goals = 18

        for i in range(num_goals):
            x = self.base_x + i+1 * distance
            y = self.base_y
            yaw = 0.0  # Facing straight

            q = quaternion_from_euler(0, 0, yaw)

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
