#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

class DockServerTestNode(Node):
    def __init__(self):
        super().__init__('dock_server_test_node')
        self.client = self.create_client(Trigger, '/dock_server_ready_check')
        
        self.get_logger().info('Waiting for the dock server...')
        if  self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Dock server service not available!')
            return

        self.get_logger().info('Dock server service available!')
        self.timer = self.create_timer(2.0, self.send_test_request)

    def send_test_request(self):
        self.get_logger().info('! Sending test request...')
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Test successful: {response.message}')
            else:
                self.get_logger().error(f'Test failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Exception while calling service: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DockServerTestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
