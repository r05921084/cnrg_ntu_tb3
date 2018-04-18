#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header

import roslaunch
import time

rospy.init_node('roslaunch_api_test', anonymous=True)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/cnrg-ntu/catkin_ws/src/cnrg_ntu_tb3/binaural_microphone/launch/basic.launch"])

rospy.on_shutdown(launch.shutdown)

launch.start()
time.sleep(30)
launch.shutdown()