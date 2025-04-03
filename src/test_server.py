#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ddr_msgs.srv import TrajTime

class VizServer(Node):
    def __init__(self):
        super().__init__('viz_server')        
        self.service = self.create_service(TrajTime, 'save_trajectory', self.serviceCallback)

    def serviceCallback(self, req, res):
        self.get_logger().info(f'Duration for saving the trajectory: {req.time}')
        
        self.target_time = self.get_clock().now().nanoseconds + int(req.time*1e9)

        self.get_logger().info(f'Target time {self.target_time}')
        self.get_logger().info(f'Current time {self.get_clock().now().nanoseconds/1e-9}')

        while self.get_clock().now().nanoseconds < self.target_time:
            self.get_logger().info(f'Target time {self.target_time}')
            self.get_logger().info(f'Current time {self.get_clock().now().nanoseconds}')
            
            
        self.get_logger().info('Data collected successfully!')

        return res
    
def main():
    rclpy.init()
    node = VizServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



