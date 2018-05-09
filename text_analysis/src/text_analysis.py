#!/usr/bin/env python
from rospy import init_node, is_shutdown
import rospy
from std_msgs.msg import String, Header
from chatbot.msg import ChatterStamped
import syntactic_extract
import gensim
import numpy as np
import rospkg

NODE_NAME = 'text_analysis'
WORDVEC_MODEL = '/wordvec_model/100features_20context_20mincount_zht'
sentence_list = []  # save the talker's talks

rospack = rospkg.RosPack()
path = rospack.get_path(NODE_NAME)


w2v_model = syntactic_extract.load_wordvec_model(path+WORDVEC_MODEL)


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


def call_back(data):
    global publisher
    rospy.loginfo(rospy.get_caller_id() + 'I heard:\n%s', data.text)
    sentence_list.append(data.text)
    # print(str(sentence_list).decode('string_escape'))
    publisher = rospy.Publisher('chatbot/output', ChatterStamped, queue_size=1)

    if data.text == 'analysis':
        sentence_list.pop()
        for s in sentence_list:
            if s[0].isalpha():
                pass
            else:
                sentence_msg = str(s) + '\n'
                syntactic_msg = 'postag_score: ' + np.array2string(syntactic_extract.pos_tag_analysis(s),
                                                                   precision=2, separator=' ', suppress_small=True) + '\n'
                semantic_msg = 'sentence_vector: ' + np.array2string(syntactic_extract.semantic_analysis(s, w2v_model),
                                                                     precision=2, separator=' ', suppress_small=True) + '\n'
                pub_to_chatbot(sentence_msg + syntactic_msg + semantic_msg)


def text_analysis():
    rospy.init_node('text_analysis', anonymous=True)
    rospy.Subscriber('chatbot/input', ChatterStamped, call_back)
    rospy.spin()


if __name__ == '__main__':
    text_analysis()
