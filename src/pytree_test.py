from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status, Access, ParallelPolicy
from py_trees.composites import Sequence, Parallel
from py_trees import logging as log_tree
from py_trees.blackboard import Client, Blackboard

class Action(Behaviour):
    def __init__(self, name, max_attempt_count=1):
        super(Action, self).__init__(name)
        self.max_attempt_count = max_attempt_count
        self.attempt_count = max_attempt_count
        self.blackboard = Client(name=self.name)
        self.blackboard.register_key(key='gripper_open',access=Access.WRITE)

    def initialise(self):
        self.attempt_count = self.max_attempt_count
        self.logger.debug(f'Action::initialize {self.name}')

    def update(self):
        self.attempt_count -= 1
        self.logger.debug(f'Action::update {self.name}')
        sleep(3)
        if not self.attempt_count:
            self.blackboard.gripper_open = True
            return Status.SUCCESS
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(f'Action::terminate {self.name} to {new_status}')

    
class Condition(Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.blackboard = Client(name=self.name)
        self.blackboard.register_key(key='battery_level',access=Access.READ)

    def initialise(self):
        self.logger.debug(f'Contion::initialize {self.name}')
    
    def update(self):
        battery = self.blackboard.battery_level
        self.logger.debug(f'Battery level: {battery}%')
        self.logger.debug(f'Contion::update {self.name}')
        sleep(1)
        return Status.SUCCESS if battery > 30 else Status.FAILURE 
    
    def terminate(self, new_status):
        self.logger.debug(f'Contion::terminate {self.name} to {new_status}')

class DrainBattery(Behaviour):
    def __init__(self, name):
        super(DrainBattery, self).__init__(name)
        self.blackboard = Client(name='BatteryDrain')
        self.blackboard.register_key(key='battery_level', access=Access.WRITE)
    
    def update(self):
        current = self.blackboard.battery_level
        self.blackboard.battery_level = max(current - 5, 0) 

        return Status.RUNNING if current > 0 else Status.SUCCESS

def make_bt():
    root = Parallel(name='Tree', policy=ParallelPolicy.SuccessOnAll())

    drainbattery = DrainBattery('drain_battery')
    
    functional_tree = Sequence(name='sequence', memory=True)

    check_battery = Condition('check_battery')
    open_gripper = Action('open_gripper', 2)
    approach_object = Action('approach_object', 2)
    close_gripper = Action('close_gripper', 3)

    functional_tree.add_children(
        [
            check_battery,
            open_gripper,
            approach_object,
            close_gripper
        ]
    )

    root.add_children([drainbattery, functional_tree])
    return root

if __name__ == '__main__':
    log_tree.level = log_tree.Level.DEBUG

    blackboard_b = Client(name='BatteryDrain')
    blackboard_b.register_key(key='battery_level', access=Access.WRITE)
    blackboard_b.battery_level = 95

    tree = make_bt()
    
    while tree.status != Status.SUCCESS and blackboard_b.battery_level > 0:
        tree.tick_once()
        sleep(1)
        print(f'Tree status : {tree.status}')
        print(f'Battery level : {blackboard_b.battery_level}')

