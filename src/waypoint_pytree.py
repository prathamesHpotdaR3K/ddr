#!/usr/bin/env python3
import rclpy
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access
from py_trees.composites import Sequence
from py_trees import logging as log_tree
from py_trees.blackboard import Client, Blackboard
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


blackboard = Client(name='nav2pose_goals')
blackboard.register_key(key='pose1', access=Access.WRITE)
blackboard.register_key(key='pose2', access=Access.WRITE)
blackboard.register_key(key='pose3', access=Access.WRITE)
blackboard.register_key(key='pose4', access=Access.WRITE)
blackboard.register_key(key='home_pose', access=Access.WRITE)

# blackboard.set( 'pose' , [x,y,w,z] )

blackboard.set('pose1', [8.8,-8.4,0.7,0.7])
blackboard.set('pose2', [8.7,8.2,0.0,-0.9])
blackboard.set('pose3', [-8.2,7.7,0.7,-0.7])
blackboard.set('pose4', [-8.1,-8.4,0.9,0.07])
blackboard.set('home_pose', [0.0,0.0,0.0,0.0])


class Action(Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def update(self):
        pose = blackboard.get(f'{self.name}')
        x = pose[0]
        y = pose[1]
        w = pose[2]
        z = pose[3]
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
            counter = counter + 1

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Reached at x:{x} y:{y} !!!')
            return Status.SUCCESS
        elif result == TaskResult.FAILED:
            print(f'Mission Failed ..|..')
            return Status.FAILURE
        elif result == TaskResult.CANCELED:
            print('Mission Canceled ..|..')
            return Status.INVALID

        

class Condition(Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

def make_bt():
    root = Sequence(name='sequence', memory=True)

    gotopose1 = Action('pose1')
    gotopose2 = Action('pose2')
    gotopose3 = Action('pose3')
    gotopose4 = Action('pose4')
    gohome = Action('home_pose')

    root.add_children([
        gotopose1,
        gotopose2,
        gotopose3,
        gotopose4,
        gohome
    ])

    return root 


if __name__ == '__main__':
    rclpy.init()   
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    log_tree.level = log_tree.Level.DEBUG

    tree = make_bt()

    while tree.status != Status.SUCCESS:
        tree.tick_once()
        print(f'Tree Status : {tree.status}')


