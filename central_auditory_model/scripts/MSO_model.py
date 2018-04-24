#!/usr/bin/env python
import numpy as np
import nengo

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage


NODE_NAME = 'nengo_mso_model'
PUB_TOPIC_NAME = 'mso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'
SAMPLE_RATE = 11025


def run_MSO_model():

    mso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImage, queue_size=1)

    def ani_cb(data):
        if data.sample_rate != SAMPLE_RATE:
            rospy.logwarn('NOT IMPLEMENT YET: sample_rate not support!')
            return
        try:
            ani_L = np.asarray(data.left_channel).reshape([data.chunk_size, data.n_subchannels])
            ani_R = np.asarray(data.right_channel).reshape([data.chunk_size, data.n_subchannels])
        except ValueError:
            rospy.logwarn('shape mismatch: %d -> %d %d' % (len(data.left_channel), data.chunk_size, data.n_subchannels))
        print ani_L.shape, ani_R.shape
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    rospy.spin()
    


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)