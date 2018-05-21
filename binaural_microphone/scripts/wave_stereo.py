#!/usr/bin/env python
# import pyaudio
import wave
import numpy as np
import time
import os

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Header
from binaural_microphone.msg import BinauralAudio


FILE_SAMPLE_RATE = 44100
FILE_PATH_AND_NAME = '/home/cnrg-ntu/PycharmProjects/Binaural-Auditory-Localization-System/SoundSplitter/coffee'
FILE_ANGLE = 0
DOWN_SAMPLE_FACTOR = 2

SUPPORTED_ANGLES = [270, 300, 330, 0, 30, 60, 90]

SAMPLE_RATE = 22050
CHUNK_SIZE= 1024

TOPIC_NAME = 'source_stream'
NODE_NAME = 'wave_stereo'
RMS_TOPIC_NAME = 'source_rms'


def wave_preprocessing(wav):
    assert wav.getsampwidth() == 2
    assert wav.getnchannels() == 1
    assert wav.getframerate() == FILE_SAMPLE_RATE
    assert wav.getframerate() / DOWN_SAMPLE_FACTOR == SAMPLE_RATE
    wav.rewind()
    wav_str = wav.readframes(wav.getnframes())
    wav_np = np.frombuffer(wav_str, dtype=np.int16)[::DOWN_SAMPLE_FACTOR]
    n_chunks = wav_np.shape[0] // CHUNK_SIZE + 1
    return np.pad(wav_np, (0, n_chunks * CHUNK_SIZE - wav_np.shape[0],), mode='constant')


def load_data(file_path_name=FILE_PATH_AND_NAME, angle=FILE_ANGLE):
    global data_L, data_R, data_start, FILE_ANGLE, FILE_PATH_AND_NAME

    if angle in SUPPORTED_ANGLES:
        FILE_ANGLE = int(angle)
        print 'angle changed to:', FILE_ANGLE
    else:
        print 'unsupported angle: %d' % angle
        return

    fn_L = '%s_%dL.wav' % (file_path_name, angle)
    fn_R = '%s_%dR.wav' % (file_path_name, angle)

    if file_path_name is not FILE_PATH_AND_NAME:
        if os.path.isfile(fn_L) and os.path.isfile(fn_R):
            FILE_PATH_AND_NAME = file_path_name
            print 'file changed to:', FILE_PATH_AND_NAME
        else:
            return

    data_L = wave_preprocessing(wave.open(fn_L, 'rb'))
    data_R = wave_preprocessing(wave.open(fn_R, 'rb'))

    data_start = 0



if __name__ == '__main__':

    global data_L, data_R, data_start

    load_data()

    try:
        rospy.init_node(NODE_NAME, anonymous=False)

        raw_pub = rospy.Publisher(TOPIC_NAME, BinauralAudio, queue_size=1)
        # raw_str_pub = rospy.Publisher('/audio_stream_raw', String, queue_size=1)  # for backward compability.

        rms_L_pub = rospy.Publisher(RMS_TOPIC_NAME + '/L', Float32, queue_size=1)
        rms_R_pub = rospy.Publisher(RMS_TOPIC_NAME + '/R', Float32, queue_size=1)

        rospy.Subscriber('~angle', Int32, lambda data: load_data(angle=data.data))
        rospy.Subscriber('~file_path_name', String, lambda data: load_data(file_path_name=data.data))

        rospy.loginfo('"%s" starts publishing to "%s".' % (NODE_NAME, TOPIC_NAME))

        rate = rospy.Rate(1.0 * SAMPLE_RATE / CHUNK_SIZE)
        
        while not rospy.is_shutdown():
            data_stop = data_start + CHUNK_SIZE

            ba = BinauralAudio(
                header=Header(
                    frame_id=NODE_NAME,
                    stamp=rospy.Time.now()
                    ),
                type='PCM Int16',
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                left_channel=data_L[data_start:data_stop],
                right_channel=data_R[data_start:data_stop]
            )
            raw_pub.publish(ba)
            # raw_str_pub.publish(raw_str)

            rms_L_pub.publish(np.sqrt(np.mean(np.square(data_L[data_start:data_stop].astype(np.float)))))
            rms_R_pub.publish(np.sqrt(np.mean(np.square(data_R[data_start:data_stop].astype(np.float)))))

            data_start = data_stop if data_stop < data_L.shape[0] else 0
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pass
