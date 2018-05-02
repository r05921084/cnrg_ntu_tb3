#!/usr/bin/env python
import numpy as np
import timeit
import threading
import nengo, nengo_dl
from delay_line import DelayLine
from signal_collector import SignalCollector

import rospy
from std_msgs.msg import String, Header, Int8
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage


NODE_NAME = 'nengo_mso_model'
PUB_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'
SAMPLE_RATE = 11025
CHUNK_SIZE = 1024
N_SUBCHANNELS = 40
MAX_DELAY = 5.
MAX_STEPS = int(MAX_DELAY * SAMPLE_RATE)

DELAY_STEPS = [0, 1, 3, 5, 7, 9, 10]
DELAY_STEPS_R = list(reversed(DELAY_STEPS))
N_DELAY_VAL = len(DELAY_STEPS)
MAXPOOLING = 100


def run_MSO_model():
    
    ani_L = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])
    ani_R = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])
    ani_L_1d = ani_L.ravel() # create an 1-D view into ani_L, must use ravel().
    ani_R_1d = ani_R.ravel() # create an 1-D view into ani_L, must use ravel().

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)

    event = threading.Event()

    def ani_cb(data):        
        if data.chunk_size != CHUNK_SIZE or data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        try:
            ani_L_1d[:] = data.left_channel
            ani_R_1d[:] = data.right_channel
        except ValueError:
            rospy.logwarn('shape mismatch: %d -> %d %d' % (len(data.left_channel), data.chunk_size, data.n_subchannels))
            return
        else:
            dl_L.update(ani_L)
            dl_R.update(ani_R)
            event.set()

    mso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImage, queue_size=1)
    mso_msg = AuditoryNerveImage(sample_rate=SAMPLE_RATE / MAXPOOLING, chunk_size=N_DELAY_VAL, n_subchannels=N_SUBCHANNELS)

    arg_pub = rospy.Publisher('max_dir', Int8, queue_size=1)

    def mso_output_cb(t, x):
        mso_msg.header = Header(stamp=rospy.Time.now())
        mso_msg.left_channel = x.ravel()
        mso_pub.publish(mso_msg)

    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    n_steps = 0

    while not rospy.is_shutdown():
        while event.wait(1.0) and not rospy.is_shutdown():
            yet_to_run = dl_R.n_steps - n_steps - CHUNK_SIZE

            if yet_to_run == 0:
                event.clear()
            elif yet_to_run > MAX_STEPS:
                rospy.logwarn('delay too much!')
                break
            elif yet_to_run < 0:
                rospy.logwarn('skip')
                event.clear()
                continue

            t2 = timeit.default_timer()
            in_L_data = dl_L.batch_view_chunk(n_steps, CHUNK_SIZE, delay_steps=DELAY_STEPS)
            in_R_data = dl_R.batch_view_chunk(n_steps, CHUNK_SIZE, delay_steps=DELAY_STEPS_R)
            print timeit.default_timer() - t2

            if in_L_data is not None and in_R_data is not None:

                out_data = in_L_data + in_R_data
                n_steps += CHUNK_SIZE

                time_mean = np.mean(out_data, axis=1)
                print time_mean.shape
                max_chan = np.max(time_mean, axis=1)
                print max_chan.shape, max_chan
                max_dir = np.argmax(max_chan)
                print max_dir
                arg_pub.publish(max_dir)

            print 'ran %d steps in %5.3f sec, %d steps yet to run.' % (CHUNK_SIZE, timeit.default_timer() - t2, yet_to_run)
        print 'main loop time out'


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)