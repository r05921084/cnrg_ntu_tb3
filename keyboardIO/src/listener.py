#!/usr/bin/env python
from rospy import init_node, is_shutdown
import rospy
from std_msgs.msg import String
import syntactic_extract
sentence_list = [] # save the talker's talks
import gensim
import numpy as np
WORDVEC_MODEL = '../wordvec_model/'
w2v_model = syntactic_extract.load_wordvec_model(WORDVEC_MODEL+'500features_20context_20mincount_zht')
print('load model success')
def call_back(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard:\n%s', data.data)
	sentence_list.append(data.data)
	# print(str(sentence_list).decode('string_escape'))
	if data.data=='analysis':
		sentence_list.pop()
		for s in sentence_list:
			print(syntactic_extract.pos_tag_analysis(s, 'jieba'))
			print(syntactic_extract.semantic_analysis(s, w2v_model))

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', String, call_back)
	rospy.spin()

if __name__ == '__main__':
	listener()