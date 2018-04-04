#!/usr/bin/env python
import numpy as np
from IPEM_pipe import IPEM_pipe

import rospy
from std_msgs.msg import String


RATE = 22050
FIFO_PATH = '/tmp/IPEM'
NODE_NAME = 'IPEM_pipe_reactor'
SUB_TOPIC_NAME = 'audio_stream_raw'
PUB_TOPIC_NAME = 'audio_stream_ani'


def reactor():
    rospy.init_node(NODE_NAME, anonymous=False)

    pub_L = rospy.Publisher(PUB_TOPIC_NAME + '/L', String, queue_size=1)
    pub_R = rospy.Publisher(PUB_TOPIC_NAME + '/R', String, queue_size=1)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    def ipem_L_cb(data):
        pub_L.publish(data)

    def ipem_R_cb(data):
        pub_R.publish(data)

    ipem_L = IPEM_pipe(new_ani_callback=ipem_L_cb, sample_frequency=RATE, fifo_path=FIFO_PATH + '/L')
    ipem_R = IPEM_pipe(new_ani_callback=ipem_R_cb, sample_frequency=RATE, fifo_path=FIFO_PATH + '/R')

    def feed_ipem(data):
        global frames
        np_data = np.fromstring(data.data, dtype=np.int16).reshape([-1, 2])
        ipem_L.feed_pcm_samples(np_data[:, 0].tostring())
        ipem_R.feed_pcm_samples(np_data[:, 1].tostring())
        rospy.loginfo(np_data.shape)

    rospy.Subscriber(SUB_TOPIC_NAME, String, feed_ipem)

    try:
        rospy.spin()
    finally:
        ipem_L.close()
        ipem_R.close()


if __name__ == '__main__':
    try:
        reactor()
    except rospy.ROSInterruptException as e:
        rospy.logerror(e)
