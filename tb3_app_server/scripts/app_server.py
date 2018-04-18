#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
import rospkg
import threading
from collections import deque

import roslaunch
import time

launches = {}
to_be_launch = deque()
new_event = threading.Event()


def launch_app(pkg, launch_file):
    path = rospack.get_path(pkg)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path + launch_file])
    launches[pkg] = launch
    launch.start()
    rospy.loginfo('launch_app' + pkg + launch_file)

def close_app(key):
    try:
        launches[key].shutdown()
        del launches[key]
        rospy.loginfo('close_app %s' % key)
    except KeyError:
        pass


def call_back(data):
    to_be_launch.append(data.data)
    new_event.set()


def close_all_launch():
    for key in launches:
        close_app(key)


def run_app_server():
    rospy.init_node('roslaunch_api_test', anonymous=True)

    global uuid, rospack
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber('chatter', String, call_back)
    rospy.on_shutdown(close_all_launch)
    rospack = rospkg.RosPack()

    while (not rospy.is_shutdown()) and new_event.wait():
        if len(to_be_launch):
            cmd = to_be_launch.popleft()

            if cmd == 'run card_game':
                launch_app('card_game', '/launch/card_game.launch')
                
            elif cmd == 'end card_game':
                close_app('card_game')

            elif cmd == 'run text_analysis':
                launch_app('keyboard_io', '/launch/text_analysis.launch')

            elif cmd == 'end text_analysis':
                close_app('keyboard_io')

            print launches


if __name__ == '__main__':
    try:
        run_app_server()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)