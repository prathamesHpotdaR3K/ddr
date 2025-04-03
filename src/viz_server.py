#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from ddr_msgs.srv import TrajTime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from datetime import datetime
from visualization_msgs.msg import MarkerArray, Marker

class VizServer(Node):
    def __init__(self):
        super().__init__('viz_server')

        self.pub_ = self.create_publisher(MarkerArray, 'diff_cont/trajectory', 10)
        self.sub_ = self.create_subscription(Odometry, 'odometry/filtered', self.markerCallback, 10)

        self.marker_arr = MarkerArray()
        self.marker_id = 0
        self.ns = "robot_trajectory" 

        self.msg_sub = self.create_subscription(Odometry, 'odometry/filtered', self.msgCallback, 10)
        self.service = self.create_service(TrajTime, 'save_trajectory', self.serviceCallback)

        self.get_logger().info('Service to save robot trajectory ready!')
        self.start_time = self.get_clock().now()

        unqtime = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.file_name = f'log_{unqtime}.csv'

        folderpath = './src/ddr/trajectory_log'
        file = os.path.join(folderpath, self.file_name)
        self.logfile = open(file, mode='a', newline='')
        self.file_writer = csv.writer(self.logfile)
        self.file_writer.writerow(['Time', 'Position-x', 'Position-y', 'Position-z','Orientation-x', 'Orientation-y', 'Orientation-w', 'Orientation-z'])

    def markerCallback(self, msg):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.ns
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  
        self.marker_arr.markers.append(marker)
        self.pub_.publish(self.marker_arr)
        self.marker_id += 1

    def msgCallback(self, msg):
        self.msgtime = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.pose = Pose()
        self.pose.position = msg.pose.pose.position
        self.pose.orientation = msg.pose.pose.orientation  

    def serviceCallback(self, req, res):
        self.get_logger().info(f'Duration for saving the trajectory: {req.time}')
        
        self.target_time = self.get_clock().now().nanoseconds + int(req.time*1e9)
                    
        self.get_logger().info('Data collected successfully!')

        self.timer = self.create_timer(0.1, self.saveTrajectory)

        res.result = f'Trajectory saved in {self.file_name}!'
        return res
    
    def saveTrajectory(self):
        current_time = self.get_clock().now().nanoseconds

        if current_time < self.target_time:
            if self.pose:
                self.file_writer.writerow([
                    self.msgtime, 
                    self.pose.position.x, 
                    self.pose.position.y, 
                    self.pose.position.z, 
                    self.pose.orientation.x, 
                    self.pose.orientation.y, 
                    self.pose.orientation.w, 
                    self.pose.orientation.z
                ])
                self.get_logger().warn('Pose added to log file!')
            else:
                self.get_logger().info('Pose data not available')
        else:
            self.timer.cancel()
            self.get_logger().info('Trajectory saved!')


def main():
    rclpy.init()
    node = VizServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



