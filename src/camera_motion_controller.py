#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class camera_motion_controller(Node):
    def __init__(self):
        super().__init__('camera_motion_controller')

        self.declare_parameter('cam_yaw_angle',1.57)

        self.cam_yaw_pub_ = self.create_publisher(Float64MultiArray, 'camera_yaw_cont/commands', 10)

        self.cam_joy_sub_ = self.create_subscription(Joy, "joy", self.joyCallback, 10)

    
    def joyCallback(self, msg):
        axis_yaw = msg.axes[0]
        cam_yaw = axis_yaw*0.785
        cam_comm = Float64MultiArray()
        cam_comm.data = [cam_yaw]
        self.cam_yaw_pub_.publish(cam_comm)


def main():
    rclpy.init()
    node = camera_motion_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


