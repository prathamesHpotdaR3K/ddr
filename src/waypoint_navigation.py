#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    poses = [
        # (x,y,w,z)
        (8.8,-8.4,0.7,0.7),
        (8.7,8.2,0.0,-0.9),
        (-8.2,7.7,0.7,-0.7),
        (-8.1,-8.4,0.9,0.07),
        (0.0,0.0,0.0,0.0)
    ]


    for (x,y,w,z) in poses:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w
        goal_pose.pose.orientation.z = z

        navigator.goToPose(goal_pose)

        counter = 0

        while not navigator.isTaskComplete():
            if counter%10==0:
                print(f'Travelling to x:{x} y:{y} ...')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Reached at x:{x} y:{y} !!!')
        elif result == TaskResult.FAILED:
            print(f'Mission Failed ..|..')
        elif result == TaskResult.CANCELED:
            print('Mission Canceled ..|..')

    rclpy.shutdown()


if __name__=='__main__':
    main()



    
    