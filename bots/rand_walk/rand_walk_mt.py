# Project: ROS2 Random Walk irobot Create3 
# Date Started: 2/22/2023
# Date Modified: 2/27/2023 
# Author Dan Blanchette
# Credit: Jordan Reed for catching my indent errors on a couple of my functions 
#         Jacob Friedberg for help with bumper config settings and suggestions for logging the robot's position

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

from builtin_interfaces.msg import Duration


import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, Dock, RotateAngle, AudioNoteSequence, NavigateToPosition
from irobot_create_msgs.msg import HazardDetectionVector, AudioNote


from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock, RLock
from rclpy.executors import MultiThreadedExecutor

# Services
from irobot_create_msgs.srv import ResetPose

# Python Libraries
import random
import math
import time


# To help with Multithreading
lock = RLock()
# snooze = time.sleep(0.5)

class Chocobo(Node):
    """
    Class to coordinate actions and subscriptions
    """
    def __init__(self, namespace):
        super().__init__('chocobo')     # type: ignore

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        cb_Subscripion = MutuallyExclusiveCallbackGroup()
        #cb_Action = cb_Subscripion
        cb_Action =MutuallyExclusiveCallbackGroup()

        # Disable Default Behavior for the Create3 Robot
        # paramName = f'/{namespace}/reflexes_enabled'
        # self.declare_parameter(paramName, False)

        # Subscription to Hazards, the callback function attached only looks for bumper hits
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data,callback_group=cb_Subscripion)

        # Action clients for movements
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',callback_group=cb_Action)
        # added dock action
        self._dock_ac = ActionClient(self, Dock, f'/{namespace}/dock', callback_group=cb_Action)
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance',callback_group=cb_Action)
        # added rotate angle topics
        self._rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle',callback_group=cb_Action )
        self._audio_note_seq = ActionClient(self, AudioNoteSequence, f'/{namespace}/audo_note_seq',callback_group=cb_Action)
        self._nav_ac = ActionClient(self, NavigateToPosition, f'/{namespace}/navigate_to_position', callback_group=cb_Action)
        
        self._reset_sc = self.create_client(ResetPose, f'/{namespace}/reset_pose')
        self.req = ResetPose.Request()
    

        # Variables
        self._goal_uuid = None
        self.pose = None
        self.bump = 0


    # bumper stuff
    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and if its
        a 'bump' message, cancel the current action. 

        For this to work, make sure you have:
        ros__parameters:
            reflexes_enabled: false
        in your Application Configuration Parameters File!!!
        '''

        # If it wasn't doing anything, there's nothing to cancel
        if self._goal_uuid is None:
            return

        # msg.detections is an array of HazardDetection from HazardDetectionVectors.
        # Other types can be gotten from HazardDetection.msg
        for detection in msg.detections:
            if detection.type == 1:   #If it is a bump
                self.get_logger().warning('HAZARD DETECTED')
                with lock: # Make this the only thing happening
                    self.get_logger().warning('CANCELING GOAL')           
                    self._goal_uuid.cancel_goal_async()
                    # Loop until the goal status returns canceled
                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        pass    
                    print('Goal canceled.')

                    self.get_logger().warning('OUCH! THAT HURTS!')
                    # back up
                    self._drive_ac.wait_for_server()
                    self.get_logger().warning('SERVER AVAILIBLE')
                    self.get_logger().warning('NOW INSIDE BACKUP PROTOCOL')
                    dr_goal = DriveDistance.Goal()
                    dr_goal.distance = -0.2
                    self.sendDriveGoal(dr_goal)
                    # waiting for rotation action server

                    self.get_logger().warning('WAITING FOR ROTATION SERVER')
                    self._rotate_ac.wait_for_server()
                    # updating dialogue
                    self.get_logger().warning('SERVER AVAILIBLE')
                    self.get_logger().warning('NOW INSIDE SPIN ROUND PROTOCOL')
                    # rotate 180 degrees
                    rot_goal = RotateAngle.Goal()
                    rot_goal.angle = (math.pi)
                    self.get_logger().warning('180 DEGREE TURN SUCCESSFUL')
                    self.sendRotationGoal(rot_goal)
                    
                    
                    self._drive_ac.wait_for_server()
                    self.get_logger().warning('SERVER AVAILIBLE')
                    self.get_logger().warning('NOW INSIDE RUNAWAY PROTOCOL')
                    dr_goal = DriveDistance.Goal()
                    dr_goal.distance = 0.5
                    self.sendDriveGoal(dr_goal)
                    self.get_logger().warning('END SUBROUTINE : SUCCESSFUL : NOW RESUMING RANDOM WALK')
                 


#--------------------Async send goal calls-----------------------------
    def sendDriveGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        
        with lock:
            drive_handle = self._drive_ac.send_goal_async(goal)
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal

            # Hold ID in case we need to cancel it
            self._goal_uuid = drive_handle.result() 



        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN: # type:ignore
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED: # type:ignore
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED: # type:ignore
                break # If the goal was canceled, stop looping otherwise loop until finished
            pass
        
        with lock:
            # Reset the goal ID, nothing should be running
            self._goal_uuid = None

    # Rotation Goal
    def sendRotationGoal(self, goal):

        with lock:
            rotation_handle = self._rotate_ac.send_goal_async(goal)
            while not rotation_handle.done():
                pass
            self._goal_uuid = rotation_handle.result()

        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN: # type:ignore
            pass # Wait until a Status has been assigned

        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED: # type:ignore
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED: # type:ignore
                break # If the goal was canceled, stop looping otherwise loop until finished
            pass

        with lock:
            # Reset the goal ID, nothing should be running
            self._goal_uuid = None

    def sendNavGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        
        with lock:
            drive_handle = self._nav_ac.send_goal_async(goal)
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal

            # Hold ID in case we need to cancel it
            self._goal_uuid = drive_handle.result() 



        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN: # type:ignore
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED: # type:ignore
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED: # type:ignore
                break # If the goal was canceled, stop looping otherwise loop until finished
            pass
        
        with lock:
            # Reset the goal ID, nothing should be running
            self._goal_uuid = None
    

#----------------------------------------------------------------------


    def drive_away(self):
        """
        Undocks robot and drives out a meter asynchronously
        """
        # list of rotations by radian values for the rotateAngle action list
        radians = [1/6, 1/4, 1/3, 1/2, 2/3, 3/4, 5/6, 1, 7/6, 5/4, 4/3, 3/2, 5/3, 7/4, 11/6, 2,
               -1/6, -1/4, -1/3, -1/2, -2/3, -3/4, -5/6, -1, -7/6, -5/4, -4/3, -3/2, -5/3, -7/4, -11/6, -2]
        
        self.pose = None

    # Freshly started, undock
        self.get_logger().warning('WAITING FOR SERVER')
    # wait until the robot server is found and ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

    # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        self._undock_ac.send_goal(undock_goal)
        self.get_logger().warning('UNDOCKED')

    # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

    # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 0.5

        self.sendDriveGoal(drive_goal)

# -----------------------------------------------------------------------------------------------------------------------
    # Get The Pose For The Robot
        self._reset_sc.call(self.req) # reset the pose
        self._pose = PoseStamped() # capture new pose
        self.get_logger().warning('THE POSE HAS BEEN CAPTURED')
# -----------------------------------------------------------------------------------------------------------------------

    # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

    # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 0.5

        self.sendDriveGoal(drive_goal)

    # send goal to async function
        self.sendDriveGoal(drive_goal)
        for i in range(0, 5):
            # send rotation goal
            self.get_logger().warning('WAITING FOR SERVER')
        
            self._rotate_ac.wait_for_server()
            # updating dialogue
            self.get_logger().warning('SERVER AVAILIBLE')
            self.get_logger().warning('ROTATING')

            rot_goal = RotateAngle.Goal()
            rot_goal.angle = (random.choice(radians) * math.pi)

            self.sendRotationGoal(rot_goal)

            self.get_logger().warning('WAITING FOR SERVER')
            self._drive_ac.wait_for_server()
            self.get_logger().warning('SERVER AVAILIBLE')
            self.get_logger().warning('DRIVING!')
        
            drive_goal2 = DriveDistance.Goal()
            drive_goal2.distance = 0.75 

            self.sendDriveGoal(drive_goal2)
            self.get_logger().warning(f'ITERATION:{i+1} SUCCESSFULLY COMPLETED')

# ------------------------------------------------------------------------------------------------------------------
        # NAVIGATE BACK TO DOCK
        nav_goal = NavigateToPosition.Goal()
        nav_goal.goal_pose = self._pose
        self.get_logger().warning(f'NAVIGATING BACK TO DOCK')

        self.sendNavGoal(nav_goal)
# ------------------------------------------------------------------------------------------------------------------

        # DOCKING REQUEST
        self.get_logger().warning('WAITING FOR SERVER')
        self._dock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILIBLE')
        self.get_logger().warning('DOCKING!')
        dock_goal = Dock.Goal()
        self._dock_ac.send_goal(dock_goal)
        self.get_logger().warning('DOCKING SUCCESSFUL')
        self.get_logger().warning('END OF PROGRAM: CTR + C TO SHUT DOWN NODES')



if __name__ == '__main__':
    rclpy.init()

    namespace = 'create3_05B9'
    c = Chocobo(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(c)

    keycom = KeyCommander([
        (KeyCode(char='r'), c.drive_away),
        ])

    print("r: Start drive_away")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        exec.shutdown()
        print("Destroying Node")
        c.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()