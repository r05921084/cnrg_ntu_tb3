#!/usr/bin/env python
import numpy as np
import timeit, time
import threading
import nengo, nengo_dl
from delay_line import DelayLine
from collections import deque

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage

# ROS
NODE_NAME = 'nengo_ic_model'
PUB_TOPIC_NAME = '/central_auditory_model/ic_stream'
SUB_MSO_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_LSO_TOPIC_NAME = '/central_auditory_model/lso_stream'

# SIGNAL
SAMPLE_RATE = 11025 // 256
CHUNK_SIZE = 1024 / 256
N_SUBCHANNELS = 40

# SIMULATION
MAX_DELAY = 5.


def run_IC_model():

    dq_mso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    dq_lso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    event = threading.Event()
    event.clear()

    def mso_cb(data):
        if data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_mso.append(data)
        event.set()
        # print 'mso_cb receive %f' % data.timecode.to_sec()

    def lso_cb(data):
        if data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_lso.append(data)
        event.set()
        # print 'lso_cb receive %f' % data.timecode.to_sec()        

    rospy.Subscriber(SUB_MSO_TOPIC_NAME, AuditoryNerveImage, mso_cb)
    rospy.Subscriber(SUB_LSO_TOPIC_NAME, AuditoryNerveImage, lso_cb)

    ic_pub = [rospy.Publisher('%s_%d' % (PUB_TOPIC_NAME, i), Float32, queue_size=1) for i in range(7)]

    rospy.loginfo('"%s" starts subscribing to "%s" and "%s".' % (NODE_NAME, SUB_MSO_TOPIC_NAME, SUB_LSO_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):
        event.clear()
        try:
            mso_msg = dq_mso.popleft()
        except IndexError:
            continue

        try:
            lso_msg = dq_lso.popleft()
        except IndexError:
            dq_mso.appendleft(mso_msg)
            continue

        # print mso_msg.timecode.to_sec(), lso_msg.timecode.to_sec()

        if mso_msg.timecode.to_sec() < lso_msg.timecode.to_sec():
            print 'skip 1 mso_msg'
            dq_lso.appendleft(lso_msg)
            continue
        elif mso_msg.timecode.to_sec() > lso_msg.timecode.to_sec():
            print 'skip 1 lso_msg'
            dq_mso.appendleft(mso_msg)
            continue

        if mso_msg.timecode.to_sec() == lso_msg.timecode.to_sec():
            print '%f match!' % mso_msg.timecode.to_sec()

            mso_data = np.array(mso_msg.left_channel).reshape(mso_msg.shape)
            lso_data = np.array(lso_msg.left_channel).reshape(lso_msg.shape)
            # print mso_data.shape, lso_data.shape

            low_freq = 10. * mso_data[:, :2, :18]
            high_freq = 5. * mso_data[:, :2, 18:] * lso_data[:, :2, 18:]

            full_freq = np.concatenate([low_freq, high_freq], axis=2)

            angle_estimate = np.mean(full_freq, axis=(1, 2))

            # print low_freq.shape, high_freq.shape, full_freq.shape, angle_estimate.shape

            for i in range(7):
                ic_pub[i].publish(angle_estimate[i])


            # for step in range(mso_msg.shape[1]):
            #     low_freq = 10. * mso_data[:, step, :18]
            #     high_freq = 5. * mso_data[:, step, 18:] * lso_data[:, step, 18:]
            #     full_freq = np.concatenate([low_freq, high_freq], axis=1)
            #     sum_of_every_angle = np.sum(full_freq, axis=1)

            #     # print full_freq.shape
            #     # print np.max(full_freq, axis=0)
            #     # (values, counts) = np.unique(np.argmax(full_freq, axis=0),return_counts=True)
            #     # ic_pub.publish(Float32(data=np.argmax(values)))
            #     for i in range(7):
            #         ic_pub[i].publish(sum_of_every_angle[i])
            #         # time.sleep(1./60)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_IC_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)