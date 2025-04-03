#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker

class LogTrajViz(Node):
    def __init__(self):
        super().__init__('log_traj_viz')

        self.pub = self.create_publisher(MarkerArray, 'diff_cont/trajectory', 10)

        self.timer = self.create_timer(0.1, self.msgCallback)

        self.filename = 'log_2025-04-03_11-45-22.csv'   #  name of log file (csv)
        self.folderpath = './src/ddr/trajectory_log'    #  folder path
        self.filepath = os.path.join(self.foldername, self.filename)

        self.pose_arr = []
        self.marker_arr = MarkerArray()
        self.ns = "robot_trajectory"
        self.marker_id = 0
        self.current_index = 0

        with open(self.filepath, 'r') as logfile:
            reader = csv.reader(logfile)
            next(reader)
            for row in reader:
                time, x, y, z, ox, oy, ow, oz = map(float, row)
                self.pose_arr.append((x, y, z))


    def msgCallback(self):
        if self.current_index >= len(self.pose_arr):
            self.get_logger().warn('Trajectory published')
            self.timer.cancel()
            return

        x, y, z = self.pose_arr[self.current_index]
        marker = Marker()
        marker.header.frame_id = "map" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.ns
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_arr.markers.append(marker)
        self.marker_id += 1

        self.current_index += 1

        self.pub.publish(self.marker_arr)
        self.get_logger().info('Publishing trajectory...')

def main():
    rclpy.init()
    node = LogTrajViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
