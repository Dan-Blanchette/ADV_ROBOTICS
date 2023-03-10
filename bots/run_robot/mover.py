#----------------------------------------
#
# @file     mover.py
# @author   Dan Blanchette
# @date     Jan 30, 2023
# @class    Robotics

# @desc     This program will send goals to the Create3 robot so the 
#           robot will move forward 1m, turn 45 degree, move 
#           forward .5m, and return home.
#
#      Credit and help for this program from: 
#              James Lasso, Garrett Wells, and Jordan Reed.
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle

from action_msgs.msg import GoalStatus

class MyNode(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self, namespace:str):
        """Initializes MyNode class with a given namespace. Has a single action client
        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('mover')

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
    node = MyNode("create3_05B9")

    # undock
    cur_goal = Undock.Goal()
    node.send_goal(Undock, 'undock', cur_goal)

    # drive for 1m
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 1.0
    node.send_goal(DriveDistance, 'drive_distance', cur_goal)

    # turn 45deg(left)
    cur_goal = RotateAngle.Goal()
    cur_goal.angle = 3.14159/4
    node.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # drive straight .5m
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 0.5
    node.send_goal(DriveDistance,'drive_distance', cur_goal)

    # turn 150 deg(left)
    cur_goal = RotateAngle.Goal()
    cur_goal.angle = 3.14159*29/36
    node.send_goal(RotateAngle, 'rotate_angle', cur_goal)

    # drive straight 1.2m
    cur_goal = DriveDistance.Goal()
    cur_goal.distance = 1.23
    node.send_goal(DriveDistance,'drive_distance', cur_goal)

    # dock robot
    cur_goal = Dock.Goal()
    node.send_goal(Dock, 'dock', cur_goal)

    # shut down node
    rclpy.shutdown()

if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()