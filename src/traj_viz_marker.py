#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped

class TrajVizMarker(Node):
    def __init__(self):
        super().__init__('traj_viz_marker')

        # Publisher/Subscriber setup
        self.pub_ = self.create_publisher(MarkerArray, 'diff_cont/trajectory', 10)
        self.sub_ = self.create_subscription(Odometry, 'odometry/filtered', self.markerCallback, 10)

        # Marker configuration
        self.marker_arr = MarkerArray()
        self.marker_id = 0
        self.ns = "robot_trajectory"  # Namespace for markers

    def markerCallback(self, msg):
        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"  # Match RViz fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.ns
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Marker position/orientation
        marker.pose = msg.pose.pose

        # Marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Marker color (RGBA)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Add to array and publish
        self.marker_arr.markers.append(marker)
        self.pub_.publish(self.marker_arr)
        
        # Increment ID for next marker
        self.marker_id += 1

def main():
    rclpy.init()
    node = TrajVizMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
