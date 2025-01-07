#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot, UndockRobot

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')


    def dock_robot(self, dock_id=""):
        self.get_logger().info("Waiting for 'DockRobot' action server...")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.dock_type = "nova_carter_dock"
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.dock_pose.header.stamp     = self.get_clock().now().to_msg()
        goal_msg.dock_pose.header.frame_id  = 'map' 
        goal_msg.dock_pose.pose.position.x  = 1.0
        goal_msg.dock_pose.pose.position.x  = 2.0
        goal_msg.dock_pose.pose.orientation.z = 1.0

        self.get_logger().info(f"Docking at ID: {dock_id}...")
        send_goal_future = self.docking_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("Docking request was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        result = self.result_future.result().result
        if result:
            self.get_logger().info("Docking completed successfully!")
        else:
            self.get_logger().error("Docking failed!")
        return result

    def _feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback.feedback}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    dock_id = 'c67f50cb-e152-4720-85cc-5eb20bd85ce8'
    node.dock_robot(dock_id)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
