#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryVisualize(Node):
    def __init__(self):
        super().__init__('trajectory_initialization')

        self.trajetory_sub = self.create_subscription(Odometry, 'diff_cont/odom', self.trajectoryCallback, 10)
        self.trajetory_pub = self.create_publisher(Path, 'diff_cont/trajectory',10)

        self.path_poses = []

    def trajectoryCallback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path_poses.append(pose)

        path = Path()
        path.header = pose.header
        path.poses = self.path_poses

        self.trajetory_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()