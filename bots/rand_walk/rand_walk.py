#----------------------------------------
#
# @file     rand_walk.py
# @author   Dan Blanchette
# @date     Jan 30, 2023
# @class    Robotics

# @desc     This program will send goals to the Create3 robot so the 
#           robot will move forward 1m, and begin a random walk routine.
#
#           The random walk allows the robot to pick a rotation degree and 
#           drive .75m in that direction.
#
#           There is also a bumper routine should it contact an object, the robot is to
#           rotate 180 degrees and then drive .5m
#
#           This program will also store cartesian information so that the robot knows
#           how to navigate back to the dock.
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle
from irobot_create_msgs.msg import HazardDetectionVector

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock
from rclpy.executors import MultiThreadedExecutor

# from action_msgs.msg import GoalStatus

import random
import math


class Chocobo_Node(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self, namespace:str):
        """Initializes MyNode class with a given namespace. Has a single action client
        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('Chocobo')

        print(f"Constructing '{namespace}' node.")
        self._namespace = namespace
        self.result = None

        self._action_client = None

    
    def send_goal(self, action_type, action_name:str, goal):
        """Sets the action client and sends the goal to the robot. Spins node until result callback is received.
        :param action_type: Action type of the action to take
        :param str action_name: name of action to append to namespace command
        :param goal: goal object with necessary parameters to complete goal
        """
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create/reuse action client with new goal info
        self._action_client = ActionClient(self, action_type, f'/{self._namespace}/{action_name}')

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal
        self.send_goal_future = self._action_client.send_goal_async(goal) #, self.feedback_callback)

        # add done callback for response
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # spin node until done
        self.result = None
        while self.result == None:
            rclpy.spin_once(self)

        # goal done
        self.get_logger().warning(f"{action_name} action done")
    
    def feedback_callback(self, feedback):
        """Callback when getting feedback. Will print feedback
        :param _type_ feedback: _description_
        """
        self.get_logger().info(f'received feedback: {feedback}')
    
    def goal_response_callback(self, future):
        """Callback when a response has been received. Calls get result callback.
        :param _type_ future: _description_
        """
        goal_handle = future.result()
        # print(f'goal handle: {goal_handle}')
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED")
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Prints result of goal. Sets self.result variable in order to stop spinning node.
        :param _type_ future: _description_
        """
        self.result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
        
def main():
    rclpy.init()
    node = Chocobo_Node("create3_05B9")

    # undock
    ud_goal = Undock.Goal()
    node.send_goal(Undock, 'undock', ud_goal)

    # drive for 1m
    drv_goal = DriveDistance.Goal()
    drv_goal.distance = 1.0
    node.send_goal(DriveDistance, 'drive_distance', drv_goal)
   # list of radian values
    radians = [1/6, 1/4, 1/3, 1/2, 2/3, 3/4, 5/6, 1, 7/6, 5/4, 4/3, 3/2, 5/3, 7/4, 11/6, 2,
               -1/6, -1/4, -1/3, -1/2, -2/3, -3/4, -5/6, -1, -7/6, -5/4, -4/3, -3/2, -5/3, -7/4, -11/6, -2]
    # random walk
    for i in range(0, 10):

      # turn a random
      rot_goal = RotateAngle.Goal()
      rot_goal.angle = (random.choice(radians) * math.pi)
      node.send_goal(RotateAngle, 'rotate_angle', rot_goal)

      # drive straight .75m
      drv_goal = DriveDistance.Goal()
      drv_goal.distance = 0.75
      node.send_goal(DriveDistance,'drive_distance', drv_goal)
      print(f'completed iteration {i + 1}\n')


    # dock robot
    dock_goal = Dock.Goal()
    node.send_goal(Dock, 'dock', dock_goal)

    # shut down node
    rclpy.shutdown()

if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()