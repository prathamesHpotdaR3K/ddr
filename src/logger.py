#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime

class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        self.sub_ = self.create_subscription(Odometry, 'odometry/filtered', self.logmsgCallback, 10)

        unqtime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.file_name = f'log_{unqtime}.csv'

        folderpath = './src/ddr/rviz'
        file = os.path.join(folderpath, self.file_name)
        self.logfile = open(file, mode='a', newline='')
        self.file_writer = csv.writer(self.logfile)
        self.file_writer.writerow(['Time', 'Position-x', 'Position-y', 'Position-z','Orientation-x', 'Orientation-y', 'Orientation-w', 'Orientation-z'])
        
    def logmsgCallback(self, msg):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        pos = msg.pose.pose.position
        orint = msg.pose.pose.orientation
        self.file_writer.writerow([time, pos.x, pos.y, pos.z, orint.x, orint.y, orint.w, orint.z])

    def destroy_node(self):
        self.logfile.close()
        return super().destroy_node()

def main():
    rclpy.init()
    node = Logger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



