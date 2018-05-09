#!/usr/bin/env/ python
import rospy
from std_msgs.msg import String, Header
from chatbot.msg import ChatterStamped


class ChatBot_IO(object):
    def __init__(self, callback, node_name='chatbot_io', publish_topic='chatbot/input', subscribe_topic='chatbot/output'):
        self.node_name = node_name
        self.callback = callback
        self.publish_topic = publish_topic
        self.subscribe_topic = subscribe_topic

        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber(self.subscribe_topic, ChatterStamped, self.__callback)
        self.publisher = rospy.Publisher(self.publish_topic, ChatterStamped, queue_size = 50)

    def __callback(self, data):
        # dict_data = {'priority': data.priority, 'source': data.source, 'text': data.text}
        self.callback(data)

    def send_to_ros(self, text, priority=0):
        msg = ChatterStamped(
            header=Header(
                stamp=rospy.Time.now()
                ),
            priority=priority,
            source=self.node_name,
            text=text,
        )
        try:
            self.publisher.publish(msg)
        except rospy.ROSInterruptException:
            pass
		
