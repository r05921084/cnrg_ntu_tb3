#!/usr/bin/env python
import numpy as np
from IPEM_pipe import IPEM_pipe

import rospy
from std_msgs.msg import String


RATE = 22050
TOPIC_NAME = 'audio_stream_raw'
NODE_NAME = 'IPEM_pipe_reactor'


def reactor():
    rospy.init_node(NODE_NAME, anonymous=False)

    pub_L = rospy.Publisher('audio_stream_ani_L', String, queue_size=1)
    pub_R = rospy.Publisher('audio_stream_ani_R', String, queue_size=1)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, TOPIC_NAME))

    def ipem_L_cb(data):
        pub_L.publish(data)

    def ipem_R_cb(data):
        pub_R.publish(data)

    ipem_L = IPEM_pipe(new_ani_callback=ipem_L_cb, sample_frequency=RATE, fifo_path='/tmp/IPEM/L')
    ipem_R = IPEM_pipe(new_ani_callback=ipem_R_cb, sample_frequency=RATE, fifo_path='/tmp/IPEM/R')

    def callback(data):
        global frames
        np_data = np.fromstring(data.data, dtype=np.int16).reshape([-1, 2])
        # rospy.loginfo(np_data.shape)
        ipem_L.feed_pcm_samples(np_data[:, 0].tostring())
        ipem_R.feed_pcm_samples(np_data[:, 1].tostring())

    rospy.Subscriber(TOPIC_NAME, String, callback)

    try:
        rospy.spin()
    finally:
        ipem_L.close()
        ipem_R.close()

if __name__ == '__main__':
    try:
        reactor()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
