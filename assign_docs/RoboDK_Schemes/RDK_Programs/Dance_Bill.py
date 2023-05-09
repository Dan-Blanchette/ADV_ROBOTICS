import time
import random as rand

from robodk.robolink import *                  # import the robolink library
RDK = Robolink()                        # connect to the RoboDK API (RoboDK starts if it has not started

robots = RDK.ItemList(ITEM_TYPE_ROBOT)
print(robots)


# tool  = RDK.Item('SCHUNK- 1321170 Co-act EGP-C 40-N-N-KTOE_ 0')                # Get an item named Tool (name in the RoboDK station tree)

robot = RDK.Item('', ITEM_TYPE_ROBOT)   # Get the first available robot


robot.Connect()
RDK.setRunMode(RUNMODE_RUN_ROBOT)
#RDK.setRunMode(RUNMODE_SIMULATE)
#frame = RDK.ItemUserPick('Select a reference frame', ITEM_TYPE_FRAME)   # Promt the user to select a reference frame
frame = RDK.Item('Frame 2', ITEM_TYPE_FRAME) 

robot.setPoseFrame(frame)
# robot.setPoseTool(tool)
rand_speed = rand.randrange(0, 100, 5)

# robot.setSpeed(1500)


def dance():
	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 1', ITEM_TYPE_TARGET)
	robot.MoveJ(target)

	target = RDK.Item('Target 2', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 3', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	#robot.setSpeed(1500, speed_joints = 20)
	target = RDK.Item('Target 4', ITEM_TYPE_TARGET)
	robot.MoveJ(target)

	target = RDK.Item('Target 5', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 6', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 7', ITEM_TYPE_TARGET)
	robot.MoveJ(target)

	target = RDK.Item('Target 8', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 9', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	target = RDK.Item('Target 10', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 11', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 12', ITEM_TYPE_TARGET)
	robot.MoveJ(target)

	target = RDK.Item('Target 13', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 14', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 15', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 16', ITEM_TYPE_TARGET)
	robot.MoveJ(target)
	
	target = RDK.Item('Target 17', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame


	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 18', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 19', ITEM_TYPE_TARGET)   # Get a target called "home"
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = rand_speed)
	target = RDK.Item('Target 20', ITEM_TYPE_TARGET)
	robot.MoveJ(target)

	robot.setSpeed(1500, speed_joints = 100)
	target = RDK.Item('Target 21', ITEM_TYPE_TARGET)   # Get a target called "turn1"
	robot.MoveJ(target)             # Move the robot to the target using the selected reference frame
for i in range(0,2):
	print("Run: ", i)
	dance()



