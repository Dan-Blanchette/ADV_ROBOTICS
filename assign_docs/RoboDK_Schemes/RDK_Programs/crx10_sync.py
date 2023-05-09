
# imports
import random as rand
from threading import RLock
from robodk.robolink import *           # import the robolink library
# connect to the RoboDK API (RoboDK starts if it has not started)
RDK = Robolink()

# common variables - config
mqtt_err = False
robot_name = "bill"
start_sync = False
lock = RLock()

try:
    import paho.mqtt.client as paho
    from paho import mqtt
except Exception as e:
    print("Error: please install 'paho.mqtt.client' module.")
    mqtt_err = True

# ---------------------------
#
# mqtt callbacks
#
# ---------------------------


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f'connected successfuly code={rc}')


def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print(f'Subscribed: {mid} {granted_qos}')


def on_publish(client, userdata, mid, properties=None):
    print(f'mid: {mid}')


def on_message(client, userdata, msg):
    print("message received")
    message_string = msg.payload.decode("ascii")

    print(f'\ntopic: {msg.topic}')
    print(f'{message_string}')

    if msg.topic == "synchronize" and robot_name not in message_string:
        print('received message from other robot')
        with lock:
            global start_sync
            start_sync = True
    else:
        print('received message from self')


def on_disconnect(client, userdata, rc, properties=None):
    if rc == paho.MQTT_ERR_SUCCESS:
        print("Disconnected successfully")
    else:
        print("Unexpected disconnect")

# ---------- connecting to mqtt broker


def connect_local_MQTT():
    print("start connecting")
    client_id = robot_name
    localMQTT = paho.Client(client_id)

    # set up call backs
    localMQTT.on_connect = on_connect
    ip_addr = "453e51c107604519a5fd673fd39f1313.s1.eu.hivemq.cloud"
    port = 8883
    username = "student"
    password = "cs383_students"
    # connect
    localMQTT.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
    localMQTT.username_pw_set(username, password)
    localMQTT.connect(host=ip_addr, port=port)

    # more callbacks
    localMQTT.on_message = on_message
    localMQTT.on_subscribe = on_subscribe
    localMQTT.on_disconnect = on_disconnect
    localMQTT.on_publish = on_publish

    return localMQTT


def publishMessage(topic, message):
    print("starting to publish")

    clientname = client._client_id
    clientname = clientname.decode(("ascii"))
    new_msg = f"{clientname}: {message}"
    client.publish(topic, new_msg, qos=1)

# -----------------
#
# robodk code
#
# -----------------


def dance(rand_speed):
    robot.setSpeed(1500, speed_joints=rand_speed)
    target = RDK.Item('Target 1', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 2', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 3', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # robot.setSpeed(1500, speed_joints = 20)
    target = RDK.Item('Target 4', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 5', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 6', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    target = RDK.Item('Target 7', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 8', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 9', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 10', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 11', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    target = RDK.Item('Target 12', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 13', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 14', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 15', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    target = RDK.Item('Target 16', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    # Get a target called "turn1"
    target = RDK.Item('Target 17', ITEM_TYPE_TARGET)
    # Move the robot to the target using the selected reference frame
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 18', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    # Get a target called "home"
    target = RDK.Item('Target 19', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=rand_speed)
    target = RDK.Item('Target 20', ITEM_TYPE_TARGET)
    robot.MoveJ(target)

    robot.setSpeed(1500, speed_joints=100)
    # Get a target called "turn1"
    target = RDK.Item('Target 21', ITEM_TYPE_TARGET)
    robot.MoveJ(target)


def start_robodk(robolink, runmode, frame_name):
    robot = robolink.Item('', ITEM_TYPE_ROBOT)
    robot.Connect()
    robolink.setRunMode(runmode)

    frame = robolink.Item(frame_name, ITEM_TYPE_FRAME)
    robot.setPoseFrame(frame)

    return robot


def move_robot(robot, tgt_names, lin_speed=1500, j_speed=180, loop=1):

    for i in range(0, len(tgt_names)*loop):             # move through targets
        robot.setSpeed(speed_linear=lin_speed,
                       speed_joints=j_speed)                  # set speed
        target = RDK.Item(tgt_names[i % len(tgt_names)], ITEM_TYPE_TARGET)
        robot.MoveJ(target)

# ---------main program----------


robot = start_robodk(RDK, RUNMODE_RUN_ROBOT, 'Bill_Base')

if not mqtt_err:
    client = connect_local_MQTT()
    client.subscribe("#")

    client.loop_start()

    print(f'started listening program for {robot_name}')

    # put robot code here
    rand_speed = rand.randrange(20, 100, 5)
    # move_robot(robot, ['squat_down'], j_speed=50)
    dance(rand_speed)

    publishMessage('synchronize', 'ready')

    while not start_sync:
        # wait for other robot to be done
        pass

    print(f'now starting synchronized swimming program')

    # synchronized robot code here
    move_robot(robot, ['Home'])
    move_robot(robot, ['Wave_Down', 'Wave_up'], loop=2)
    move_robot(robot, ['Swing', 'Straight_up'])
    move_robot(robot, ['Home'])

    client.loop_stop()
    client.disconnect()
