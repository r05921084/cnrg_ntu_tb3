#!/usr/bin/env python
import numpy as np
import time

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Header
from binaural_microphone.msg import BinauralAudio


SAMPLE_RATE = 22050
CHUNK_SIZE= 1024
FRAME_TIME = 1.0 * CHUNK_SIZE / SAMPLE_RATE
CHANNELS = 2

TOPIC_NAME = 'source_stream'
NODE_NAME = 'sine_sweep'

RMS_TOPIC_NAME = 'source_rms'

FREQ_START = 1
FREQ_STOP = 20000
N_STEP = 1000
AMPLITUDE = 0.5

# for the 16-bit PCM audio signal -32768 to 32767, scaled to [-1 to 1)
# max_db = 20*np.log10(abs(+-1)) = 0 dB
# min_db = 20*np.log10(1/32767) ~= -90.3 dB


def pcm_sine_generator():
    freq = np.geomspace(FREQ_START, FREQ_STOP, N_STEP)
    data = np.zeros([CHUNK_SIZE, CHANNELS], dtype=np.int16)
    t = np.linspace(0, FRAME_TIME, CHUNK_SIZE, endpoint=False)    
    print freq
    print t
    while True:
        for i in range(N_STEP):
            f = freq[i]
            # f = 2000
            data[:, 0] = AMPLITUDE * 32767 * np.sin(2 * np.pi * f * t)
            data[:, 1] = AMPLITUDE * 32767 * np.cos(2 * np.pi * f * t)
            t += FRAME_TIME
            rospy.loginfo('t = %fsec, f = %fHz' % (t[-1], f))
            yield data


if __name__ == '__main__':

    try:
        rospy.init_node(NODE_NAME, anonymous=False)

        raw_pub = rospy.Publisher(TOPIC_NAME, BinauralAudio, queue_size=1)
        rms_L_pub = rospy.Publisher(RMS_TOPIC_NAME + '/L', Float32, queue_size=1)
        rms_R_pub = rospy.Publisher(RMS_TOPIC_NAME + '/R', Float32, queue_size=1)
        # rms_D_pub = rospy.Publisher(RMS_TOPIC_NAME + '/debug', Float32, queue_size=1)

        rospy.loginfo('"%s" starts publishing to "%s".' % (NODE_NAME, TOPIC_NAME))

        rate = rospy.Rate(1. / FRAME_TIME)

        sine_gen = pcm_sine_generator()
        
        while not rospy.is_shutdown():
            np_data = sine_gen.next()
            ba = BinauralAudio(
                header=Header(
                    frame_id=NODE_NAME,
                    stamp=rospy.Time.now()
                    ),
                type='PCM Int16',
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                left_channel=np_data[:, 0],
                right_channel=np_data[:, 1]
            )
            raw_pub.publish(ba)

            rms_L_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 0].astype(np.float)))))
            rms_R_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 1].astype(np.float)))))
            # rms_D_pub.publish(t[-1])
            rate.sleep()

    except rospy.ROSInterruptException:
        pass