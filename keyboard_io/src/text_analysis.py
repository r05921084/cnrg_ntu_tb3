#!/usr/bin/env python
from rospy import init_node, is_shutdown
import rospy
from std_msgs.msg import String
import syntactic_extract
sentence_list = [] # save the talker's talks
import gensim
import numpy as np
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('keyboard_io')

w2v_model = syntactic_extract.load_wordvec_model(path+'/wordvec_model/100features_20context_20mincount_zht')
print('load model success')

def call_back(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard:\n%s', data.data)
	sentence_list.append(data.data)
	# print(str(sentence_list).decode('string_escape'))
	if data.data=='analysis':
		sentence_list.pop()
		for s in sentence_list:
			if s[0].isalpha():
				pass
			else:
				print(syntactic_extract.pos_tag_analysis(s))
				print(syntactic_extract.semantic_analysis(s, w2v_model))

def text_analysis():
	rospy.init_node('text_analysis', anonymous=True)
	rospy.Subscriber('chatter', String, call_back)
	rospy.spin()

if __name__ == '__main__':
	text_analysis()
