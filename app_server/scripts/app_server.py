#!/usr/bin/env python
import rospy, rospkg
from std_msgs.msg import String, Header
from chatbot.msg import ChatterStamped

import threading
from collections import deque

import roslaunch
import time


NODE_NAME = 'app_server'


supported_cmd = {}
supported_cmd['card_game'] = ('card_game', '/launch/card_game.launch')
supported_cmd['text_analysis'] = ('text_analysis', '/launch/text_analysis.launch')
supported_cmd['ipem_visual_split'] = ('ipem_module', '/launch/visual_split.launch')
supported_cmd['acoustic_anomaly'] = ('acoustic_anomaly', '/launch/publish_acoustic_anomaly.launch')
supported_cmd['video_falling_detection'] = ('video_falling_detection', '/launch/test_launch.launch')

supported_op = ['run', 'end']


def pub_to_chatbot(text, priority=0):
    msg = ChatterStamped(
        header=Header(
            stamp=rospy.Time.now()
            ),
        priority=priority,
        source=NODE_NAME,
        text=text,
    )
    publisher.publish(msg)


def launch_app(key, pkg=None, launch_file=None):
    if pkg is None or launch_file is None:
        try:
            (pkg, launch_file) = supported_cmd[key]
        except KeyError:
            rospy.logwarn('not supported app: %s' % key)
            pub_to_chatbot('not supported app: %s' % key)
            return

    path = rospack.get_path(pkg)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path + launch_file])
    try:
        launch.start()        
    except roslaunch.RLException as e:
        rospy.logwarn(e)
        pub_to_chatbot('expection raised in launching app: %s' % key)
    else:
        launched_app[key] = launch
        rospy.loginfo('launch_app: [%s] %s, %s' % (key, pkg, path + launch_file))
        pub_to_chatbot('successfully launched app: %s' % key)
            


def close_app(key):
    try:
        launched_app[key].shutdown()
        del launched_app[key]
        rospy.loginfo('close_app [%s]' % key)
        pub_to_chatbot('successfully closed app: %s' % key)
    except KeyError:
        pass


def new_cmd_callback(data):
    cmd = [key for key in supported_cmd if key in data.text]
    op = [op for op in supported_op if op in data.text]
    if len(cmd) and len(op):        
        cmd_queue.append((cmd[0], op[0]))
        new_event.set()


def close_all_app():
    for key in launched_app:
        close_app(key)
    rospy.loginfo('close_all_app')
    pub_to_chatbot('successfully closed all app.')



def run_app_server():
    global uuid, rospack, launched_app, cmd_queue, new_event, publisher

    launched_app = {}
    cmd_queue = deque()
    new_event = threading.Event()

    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.on_shutdown(close_all_app)
    rospack = rospkg.RosPack()
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber('chatbot/input', ChatterStamped, new_cmd_callback)
    publisher = rospy.Publisher('chatbot/output', ChatterStamped, queue_size=1)

    while not rospy.is_shutdown():
        while new_event.wait(0.5):
            cmd, op = cmd_queue.popleft()
            new_event.clear()

            if op == 'run':
                launch_app(cmd)
            elif op == 'end':
                close_app(cmd)

            rospy.logdebug(launched_app)


if __name__ == '__main__':
    try:
        run_app_server()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
