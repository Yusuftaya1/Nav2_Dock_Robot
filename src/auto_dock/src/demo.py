#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
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
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
def main():
    rclpy.init()
    tester = DockingTester()
    dock_id = 'test_dock1'
    tester.dockRobot(dock_id=dock_id)

if __name__ == '__main__':
    main()