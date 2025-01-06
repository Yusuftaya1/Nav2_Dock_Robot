#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from opennav_docking_msgs.action import DockRobot
from rclpy.action import ActionClient

class DockRobotClient(Node):
    def __init__(self):
        super().__init__('dock_robot_client')
        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')

    def _feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback}")

    def dock_robot(self, dock_id=""):
        self.get_logger().info("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.dock_pose.header.stamp     = self.get_clock().now().to_msg()
        goal_msg.dock_pose.header.frame_id  = dock_id
        goal_msg.dock_pose.pose.position.x  = 1.0
        goal_msg.dock_pose.pose.position.y  = 1.0

        self.get_logger().info(f'Docking at ID: {dock_id}...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg, self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        return True

def main(args=None):
    rclpy.init(args=args)

    dock_client = DockRobotClient()
    dock_id = 'c67f50cb-e152-4720-85cc-5eb20bd85ce8'  # Example dock ID
    success = dock_client.dock_robot(dock_id)

    if success:
        dock_client.get_logger().info('Docking successful!')
    else:
        dock_client.get_logger().info('Docking failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
