#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from opennav_docking_msgs.action import DockRobot, UndockRobot

class DockingTester(Node):
    def __init__(self):
        super().__init__(node_name='docking_tester')
        self.goal_handle = None
        self.feedback = None

        self.docking_client = ActionClient(self, DockRobot,'dock_robot')
        # self.undocking_client = ActionClient(self, UndockRobot,'undock_robot')

    def dockRobot(self,dock_id = ""):
        """Send a `DockRobot` action request."""
        print("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=3.0):
            print('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.navigate_to_staging_pose = False
        goal_msg.use_dock_id = True # database kullanılırsa TRUE yapılabilir
        goal_msg.dock_id = dock_id 

        print('Docking to İD: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Docking request was rejected!')
            return False

        return True
    
    def dockRobotwpose(self, dock_pose, dock_type = ""):
        """Send a `DockRobot` action request."""
        print("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            print('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False
        # goal_msg.dock_id = dock_id  # if wanting to use ID instead
        goal_msg.dock_pose = dock_pose
        goal_msg.dock_type = dock_type
        # goal_msg.navigate_to_staging_pose = True  # if want to navigate before staging

        print('Docking at pose: ' + str(dock_pose) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Docking request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
def main():
    rclpy.init()
    tester = DockingTester()
    dock_id = 'test_dock1'

    dock_pose = PoseStamped()
    dock_pose.header.stamp = tester.get_clock().now().to_msg()
    dock_pose.header.frame_id = "base_link"
    dock_pose.pose.position.x = 2.0
    dock_pose.pose.position.y = 4.0

    tester.dockRobot(dock_id=dock_id)
    #tester.dockRobotwpose(dock_pose)


if __name__ == '__main__':
    main()