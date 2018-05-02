#!/usr/bin/env python
import rospy
import numpy as np

def model(pic, pub):

    if pic.mean()<30:
        # rospy.loginfo('ABNORMALL!!!!!!!!!!!!!!')
        pub.publish('ABNORMALL!!!!!!!!!!!!!!')
    else:
        #rospy.loginfo('========NORMALL========')
        pub.publish('========NORMALL========')