#!/usr/bin/env python

import rospy
from std_msgs.msg import String



def speech2text():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # pub = rospy.Publisher('dialog', String)
    rospy.init_node('s2t', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        sentence = raw_input()
        rospy.loginfo(sentence)
        pub.publish(sentence)
        # rate.sleep()


if __name__ == '__main__':
    try:
        speech2text()
    except rospy.ROSInterruptException:
        pass
