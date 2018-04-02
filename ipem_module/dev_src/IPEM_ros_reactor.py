#!/usr/bin/env python
import wave
import numpy as np
from IPEM_subproc import IPEM_subproc
import time
import subprocess
import os
import threading


import rospy
from std_msgs.msg import String


frames = []
event_main_loop = threading.Event()


def callback(data):
    global frames
    np_data = np.fromstring(data.data, dtype=np.int16).reshape([-1, 2])
    # rospy.loginfo(np_data.shape)
    frames.append(np_data)
    if len(frames) > 10:
        event_main_loop.set()


def reactor():
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Subscriber(TOPIC_NAME, String, callback)
    pub_L = rospy.Publisher('audio_stream_ani_L', String, queue_size=1)
    pub_R = rospy.Publisher('audio_stream_ani_R', String, queue_size=1)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, TOPIC_NAME))

    if os.path.isdir(TEMP_FILE_DIR):
        print('%s already exist.' % TEMP_FILE_DIR)
    else:
        try:
            subprocess.check_call(['mkdir', TEMP_FILE_DIR])
            print('mkdir %s... ok!' % TEMP_FILE_DIR)
        except subprocess.CalledProcessError:
            print('mkdir %s... ERROR!' % TEMP_FILE_DIR)
            return

    ipem_L = IPEM_subproc(inInputFileName='input_L.wav', inOutputFileName='output_L.ani',
                          inInputFilePath=TEMP_FILE_DIR, inOutputFilePath=TEMP_FILE_DIR,
                          skip_check=True, sample_frequency=RATE)
    ipem_R = IPEM_subproc(inInputFileName='input_R.wav', inOutputFileName='output_R.ani',
                          inInputFilePath=TEMP_FILE_DIR, inOutputFilePath=TEMP_FILE_DIR,
                          skip_check=True, sample_frequency=RATE)


    while not rospy.is_shutdown() and event_main_loop.wait(3.0):
        event_main_loop.clear()

        global frames
        t1 = time.time()
        all_frames = np.vstack(frames)
        frames = []

        waveFile = wave.open(TEMP_FILE_DIR + '/' + 'input_L.wav', 'wb')
        waveFile.setnchannels(1)
        waveFile.setsampwidth(2)
        waveFile.setframerate(RATE)
        waveFile.writeframes(all_frames[:, 0].tostring())
        waveFile.close()

        waveFile = wave.open(TEMP_FILE_DIR + '/' + 'input_R.wav', 'wb')
        waveFile.setnchannels(1)
        waveFile.setsampwidth(2)
        waveFile.setframerate(RATE)
        waveFile.writeframes(all_frames[:, 1].tostring())
        waveFile.close()

        # event_L.set()
        # event_R.set()

        ani_L, log_L = ipem_L.process()
        ani_R, log_R = ipem_R.process()

        pub_L.publish(ani_L.tostring())
        pub_R.publish(ani_R.tostring())

        print('main_loop: ', time.time() - t1)

if __name__ == '__main__':
    RATE = 22050

    TOPIC_NAME = 'audio_stream_raw'
    NODE_NAME = 'IPEM_reactor'

    TEMP_FILE_DIR = '/tmp/%s' % NODE_NAME

    reactor()




    # event_L = threading.Event()
    # event_R = threading.Event()
    #
    # def ipem_worker_L():
    #     ipem_L = IPEM_subproc(inInputFileName='input_L.wav', inOutputFileName='output_L.ani',
    #                           inInputFilePath=TEMP_FILE_DIR, inOutputFilePath=TEMP_FILE_DIR,
    #                           skip_check=True, sample_frequency=RATE)
    #     while not rospy.is_shutdown() and event_L.wait(3.0):
    #         event_L.clear()
    #         ani_L, log_L = ipem_L.process()
    #         pub_L.publish(ani_L.tostring())
    #         print('ipem_worker_L: ', time.time() - t1)
    #
    # def ipem_worker_R():
    #     ipem_R = IPEM_subproc(inInputFileName='input_R.wav', inOutputFileName='output_R.ani',
    #                           inInputFilePath=TEMP_FILE_DIR, inOutputFilePath=TEMP_FILE_DIR,
    #                           skip_check=True, sample_frequency=RATE)
    #     while not rospy.is_shutdown() and event_R.wait(3.0):
    #         event_R.clear()
    #         ani_R, log_R = ipem_R.process()
    #         pub_R.publish(ani_R.tostring())
    #         print('ipem_worker_R: ', time.time() - t1)
    #
    # threading.Thread(target=ipem_worker_L, name='ipem_worker_L').start()
    # threading.Thread(target=ipem_worker_R, name='ipem_worker_R').start()