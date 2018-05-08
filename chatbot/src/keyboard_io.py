#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from chatbot.msg import ChatterStamped

NODE_NAME = 'keyboard_io'
PUBLISH_TOPIC = 'chatbot/input'
SUBSCRIBE_TOPIC = 'chatbot/output'


def chatter_callback(data):
	rospy.loginfo('(%d)[%s] said: "%s"' % (data.priority, data.source, data.text))
	

def talker():
	rospy.init_node(NODE_NAME, anonymous=True)
	pub = rospy.Publisher(PUBLISH_TOPIC, ChatterStamped, queue_size = 50)
	rospy.Subscriber(SUBSCRIBE_TOPIC, ChatterStamped, chatter_callback)
	
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		print('say something: ')
		text = raw_input()
		if text=='q':
			break
		else:
			rospy.loginfo('you said: "%s"' % text)
			msg = ChatterStamped(
				header=Header(
					stamp=rospy.Time.now()
					),
				priority=0,
				source=NODE_NAME,
				text=text,
			)			
			pub.publish(msg)
			rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass