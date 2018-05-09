#!/usr/bin/env python

import os
import time
import rospy
import rospkg
import numpy as np
import threading

from keras.models import load_model
from python_speech_features import mfcc
from sklearn.metrics.pairwise import cosine_similarity

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from chatbot.msg import ChatterStamped
from binaural_microphone.msg import BinauralAudio


rospack = rospkg.RosPack()
path = rospack.get_path('acoustic_anomaly')
NODE_NAME = 'acoustic_anomaly_detector'
frames = []
fs = 22050
event = threading.Event()

### load data from files
param_path = os.path.join(path, 'param')
embedded_model = load_model(os.path.join(param_path, 'embedded_model.h5'))
embedded_model._make_predict_function()
mean = np.load(os.path.join(param_path, 'NPYs/mean.npy'))
std = np.load(os.path.join(param_path, 'NPYs/std.npy'))
normal_SP = np.load(os.path.join(param_path, 'NPYs/Normal.npy')).reshape(1, -1)
abnormal_SP = np.load(os.path.join(param_path, 'NPYs/Abnormal.npy')).reshape(1, -1)


def pub_to_chatbot(text, priority=0):
    msg = ChatterStamped(
        header=Header(
            stamp=rospy.Time.now()
            ),
        priority=priority,
        source=NODE_NAME,
        text=text,
    )
    publisher.publish(msg)


def storing_from_array(data):
    global frames
    frames += data.left_channel
    assert data.sample_rate == fs, 'sample rate mismatch!'
    if len(frames) > 2205:
        event.set()


def model():
    global frames
    ### take the first 0.1(sec)*fs(Hz) points to make prediction
    if len(frames) < 2205:
        return None
    else:
        sig = np.array(frames[0:2205])
        del frames[0:2205]
        # assert sig.shape == (2205,), 'an audio segment must have 0.1(sec)*fs(Hz) points'

        ### detection model here!!
        rospy.loginfo('start model')
        ## data preprocess: audio to mfcc
        mfccs = mfcc(sig, fs, numcep=26, winlen=0.025, winstep=0.01)  # shape = (9, 26)
        mfccs = mfccs.reshape(1, -1)  # shape = (1, 234)
        mfccs = (mfccs - mean)/std
        embedded_vector = embedded_model.predict(mfccs)  # shape = (1, 32)

        ## calculate similarity of target_SP and embedded_vector, both with shape = (1, 32)
        ## cosine_similarity returns an array with shape = (1, 1)

        sim_normal = cosine_similarity(normal_SP, embedded_vector)[0, 0] 
        sim_abnormal = cosine_similarity(abnormal_SP, embedded_vector)[0, 0]
        rospy.loginfo('end model')

        if sim_abnormal > sim_normal: 
            return 'ABNORMAL !!!'
        else: 
            return 'NORMAL'


def get_raw_audio():
    TOPIC_NAME = '/binaural_audio/source_stream'
    rospy.Subscriber(TOPIC_NAME, BinauralAudio, storing_from_array)


def talker():
    global publisher
    publisher = rospy.Publisher('chatbot/output', ChatterStamped, queue_size=1)
    pub_to_chatbot(text='start monitoring ...')

    pub = rospy.Publisher('Acoustic_Anomaly', Float64MultiArray, queue_size=10)
    array = Float64MultiArray()

    def process():
        ab = model()
        if ab:
            if ab == 'NORMAL':
                array.data = [0]
            else:
                array.data = [1]
            rospy.loginfo(ab)
            pub.publish(array)
            pub_to_chatbot(text=ab)

    while not rospy.is_shutdown():
        while event.wait(1.0):
            event.clear()
            threading.Thread(target=process).start()
        print('timeout!')


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        get_raw_audio()
        talker()
    except rospy.ROSInterruptException:
        pass
