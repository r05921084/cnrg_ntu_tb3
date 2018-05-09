#!/usr/bin/env python
import rospy
import numpy as np

def model(pic):

    if pic.mean()<30:
        # rospy.loginfo('ABNORMALL!!!!!!!!!!!!!!')
        return 'ABNORMALL!!!!!!!!!!!!!!'
    else:
        #rospy.loginfo('========NORMALL========')
        return '========NORMALL========'
