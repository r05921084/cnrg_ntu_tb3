#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():

	pub = rospy.Publisher('chatter', String, queue_size = 100)	
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		print('say something: ')
		something = raw_input()
		if something=='q':
			break
		else:
			rospy.loginfo(something)
			pub.publish(something)
			rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass