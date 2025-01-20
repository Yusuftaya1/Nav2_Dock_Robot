#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot, UndockRobot
import time 

class DockingTester(Node):
    def __init__(self):
        super().__init__(node_name='docking_tester')
        self.goal_handle = None
        self.feedback = None

        self.docking_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self, UndockRobot, 'undock_robot')

    def dockRobot(self, dock_id=""):
        """Send a `DockRobot` action request."""
        print("Waiting for 'DockRobot' action server")
        while not self.docking_client.wait_for_server(timeout_sec=3.0):
            print('"DockRobot" action server not available, waiting...')

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.navigate_to_staging_pose = False
        goal_msg.dock_id = dock_id 

        print('Docking to ID: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            print('Docking request was rejected!')
            return False
        
        print("Docking request accepted.")
        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        result = self.result_future.result().result
        if result.success:
            print("Docking completed successfully.")
            return True
        else:
            print("Docking failed (result).")
            return False

    def undockRobot(self, dock_type):
        """Send a `UndockRobot` action request."""
        print("Waiting for 'UndockRobot' action server")
        while not self.undocking_client.wait_for_server(timeout_sec=3.0):
            print('"UndockRobot" action server not available, waiting...')

        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type

        print('Undocking from dock of type: ' + str(dock_type) + '...')
        send_goal_future = self.undocking_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Undocking request was rejected!')
            return False

        print("Undocking request accepted.")
        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        result = self.result_future.result().result
        if result.success:
            print("Undocking completed successfully.")
            return True
        else:
            print("Undocking failed.")
            return False

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        print("Feedback received: ", self.feedback)


def main():
    rclpy.init()
    tester = DockingTester()
    dock_idd = 'loading_dock'
    dock_type = 'sattva_apl_dock'

    tester.dockRobot(dock_id=dock_idd)
    #print("Waiting for 4 seconds before undocking...")
    #time.sleep(4)

    #tester.undockRobot(dock_type=dock_type)

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()