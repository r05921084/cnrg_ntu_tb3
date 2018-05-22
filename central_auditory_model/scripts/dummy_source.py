#!/usr/bin/env python
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from ipem_module.msg import AuditoryNerveImage

SAMPLE_RATE = 11025
N_SUBCHANNELS = 40
CHUNK_SIZE= 1024

if __name__ == '__main__':
    try:
        rospy.init_node('dummy_ani_source', anonymous=False)
        ani_pub = rospy.Publisher('/ipem_module/apm_stream', AuditoryNerveImage, queue_size=1)

        rate = rospy.Rate(1.0 * SAMPLE_RATE / CHUNK_SIZE)

        ani = AuditoryNerveImage(
                header=Header(
                    stamp=rospy.Time.now()
                ),
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                n_subchannels=N_SUBCHANNELS,
                left_channel=np.ones([CHUNK_SIZE * N_SUBCHANNELS]),
                right_channel=np.ones([CHUNK_SIZE * N_SUBCHANNELS]),
            )

        while not rospy.is_shutdown():
            # ani = AuditoryNerveImage(
            #     header=Header(
            #         stamp=rospy.Time.now()
            #     ),
            #     sample_rate=SAMPLE_RATE,
            #     chunk_size=CHUNK_SIZE,
            #     n_subchannels=N_SUBCHANNELS,
            #     left_channel=np.ones([CHUNK_SIZE * N_SUBCHANNELS]),
            #     right_channel=np.ones([CHUNK_SIZE * N_SUBCHANNELS]),
            # )
            ani.header = Header(stamp=rospy.Time.now())
            ani_pub.publish(ani)
            print ani.header.stamp

            rate.sleep()


    except rospy.ROSInterruptException as e:
        rospy.logerr(e)