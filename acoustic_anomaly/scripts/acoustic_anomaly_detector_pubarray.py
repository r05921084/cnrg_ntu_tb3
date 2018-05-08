#!/usr/bin/env python

import os
import time
import rospy
import rospkg
import numpy as np
import threading

from keras.models import load_model
from python_speech_features import mfcc

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from binaural_microphone.msg import BinauralAudio


rospack = rospkg.RosPack()
path = rospack.get_path('acoustic_anomaly')


def storing_from_str(data):
    global frames
    np_data = np.fromstring(data.data, dtype=np.int16).reshape([-1, 2])
    frames.append(np_data)


def storing_from_array(data):
    global frames
    np_data = np.array(data.left_channel, dtype=np.int16)  # choose left channel only
    frames.append(np_data)


def model():
    global frames
    try:
        np_data = np.vstack(frames)
    except ValueError:
        pass

    frames = []
    ### make sure that np_data has 0.1(sec)*fs(Hz) points
    # required_points = int(0.1*fs)

    ### Anomaly detection model here!!
    rospy.loginfo('start model')
    ## data preprocess: audio to mfcc
    # mfccs = mfcc(sig, fs, numcep=26, winlen=0.025, winstep=0.01)  # shape = (9, 26)
    # mfccs = mfccs.reshape(1, -1)  # shape = (1, 234)
    # mfccs = (mfccs - mean)/std

    # embedded_vector = embedded_model.predict(mfccs)  # shape = (1, 32)

    ## calculate similarity of target_SP and embedded_vector, both with shape = (1, 32)
    ## cosine_similarity returns an array with shape = (1, 1)

    # sim_normal = cosine_similarity(normal_SP, embedded_vector)[0, 0] 
    # sim_abnormal = cosine_similarity(abnormal_SP, embedded_vector)[0, 0]
    # if sim_abnormal > sim_normal: return 'abnormal'
    # else: return 'normal'

    time.sleep(0.1)
    rospy.loginfo('end model')
    return 'normal'


def get_raw_audio():
    # TOPIC_NAME = 'audio_stream_raw'
    # rospy.Subscriber(TOPIC_NAME, String, storing_from_str)
    TOPIC_NAME = 'source_stream'
    rospy.Subscriber(TOPIC_NAME, BinauralAudio, storing_from_array)


def talker():
    array = Float64MultiArray()
    pub = rospy.Publisher('Acoustic_Anomaly', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        def process():
            ab = model()
            rospy.loginfo(ab)
            if ab == 'normal':
            	array.data = [0]
            else:
            	array.data = [1]
            # data.append(ab)
            pub.publish(array)

        threading.Thread(target=process).start()
        rate.sleep()

frames = []
fs = 22050
### load data from files
param_path = os.path.join(path, 'param')
embedded_model = load_model(os.path.join(param_path, 'embedded_model.h5'))
mean = np.load(os.path.join(param_path, 'NPYs/mean.npy'))
std = np.load(os.path.join(param_path, 'NPYs/std.npy'))
normal_SP = np.load(os.path.join(param_path, 'NPYs/Normal.npy')).reshape(1, -1)
abnormal_SP = np.load(os.path.join(param_path, 'NPYs/Abnormal.npy')).reshape(1, -1)


if __name__ == '__main__':
    try:
        rospy.init_node('acoustic_anomaly_detector', anonymous=True)
        get_raw_audio()
        talker()
    except rospy.ROSInterruptException:
        pass
