#!/usr/bin/env python
import pyaudio
# import wave
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32


def microphone_recorder():
    rospy.init_node(NODE_NAME, anonymous=False)
    raw_pub = rospy.Publisher(TOPIC_NAME, String, queue_size=10)
    rms_L_pub = rospy.Publisher('audio_rms_L', Float32, queue_size=1)
    rms_R_pub = rospy.Publisher('audio_rms_R', Float32, queue_size=1)

    rospy.loginfo('"%s" starts publishing to "%s".' % (NODE_NAME, TOPIC_NAME))

    while not rospy.is_shutdown():
        data = stream.read(CHUNK)
        rospy.loginfo(len(data))
        raw_pub.publish(data)
        np_data = np.fromstring(data, dtype=np.int16).reshape([-1, 2]).astype(np.float)
        rms_L_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 0]))))
        rms_R_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 1]))))


if __name__ == '__main__':

    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 22050
    CHUNK = 1024
    RECORD_SECONDS = 1

    TOPIC_NAME = 'audio_stream_raw'
    NODE_NAME = 'microphone_recorder'

    audio = pyaudio.PyAudio()

    # start Recording
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)
    try:
        microphone_recorder()
    except rospy.ROSInterruptException:
        pass
    finally:
        # stop Recording
        stream.stop_stream()
        stream.close()
        audio.terminate()
        rospy.loginfo('stream.stop_stream()')
        rospy.loginfo('stream.close()')
        rospy.loginfo('audio.terminate()')
