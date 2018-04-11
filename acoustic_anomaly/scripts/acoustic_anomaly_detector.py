#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import threading
import time
import numpy as np


def callback(data):
    global frames
    np_data = np.fromstring(data.data, dtype=np.int16).reshape([-1, 2])
    rospy.loginfo(np_data.shape)
    frames.append(np_data)

def model():
    global frames
    np_data = np.vstack(frames)
    frames = []
    ### Anomaly detection model here!!
    print 'start model'
    time.sleep(0.15)
    print 'end model'
    return 'normal'
    ### 



def get_raw_audio():
    TOPIC_NAME = 'audio_stream_raw'
    rospy.Subscriber(TOPIC_NAME, String, callback)
    # while True:
    #     pass
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()



def talker():
    pub = rospy.Publisher('Acoustic_Anomaly', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "Normal Sound %s" % rospy.get_time()
        def process():
            ab = model()
            rospy.loginfo(ab)
            pub.publish(ab)

        threading.Thread(target=process).start()
        rate.sleep()

frames = []



if __name__ == '__main__':
    try:
        rospy.init_node('acoustic_anomaly_detector', anonymous=True)
        get_raw_audio()
        talker()
    except rospy.ROSInterruptException:
        pass
