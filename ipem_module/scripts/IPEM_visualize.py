#!/usr/bin/env python
import numpy as np

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage


NODE_NAME = 'ipem_visualizer'
SUB_TOPIC_NAME = 'apm_stream'
PUB_TOPIC_NAME = '/visualization_marker'


def hsva_to_rgba(h, s, v, a=1.0):
    if s == 0.0:
        return v, v, v, a
    i = int(h*6.0) # XXX assume int() truncates!
    f = (h*6.0) - i
    p = v*(1.0 - s)
    q = v*(1.0 - s*f)
    t = v*(1.0 - s*(1.0-f))
    i = i%6
    if i == 0:
        return v, t, p, a
    if i == 1:
        return q, v, p, a
    if i == 2:
        return p, v, t, a
    if i == 3:
        return p, q, v, a
    if i == 4:
        return t, p, v, a
    if i == 5:
        return v, p, q, a


def visualizer():
    rospy.init_node(NODE_NAME, anonymous=False)
    marker_publisher = rospy.Publisher(PUB_TOPIC_NAME, Marker, queue_size=1)
    
    def ani_cb(data):
        points_L = [Point(0.01 * i - data.chunk_size * 0.005, 0.1 * j + 0.5, data.left_channel[i * 40 + j] * 10) for j in range(data.n_subchannels) for i in range(0, data.chunk_size, 10)]
        points_R = [Point(0.01 * i - data.chunk_size * 0.005, 0.1 * j - 0.5 - data.n_subchannels * 0.1, data.right_channel[i * 40 + j] * 10) for j in range(data.n_subchannels) for i in range(0, data.chunk_size, 10)]
        colors_L = [ColorRGBA(*hsva_to_rgba(0.67 - 0.67 * (float(j) / data.n_subchannels), 1.0, np.clip(data.left_channel[i * 40 + j] * 10, 0., 1.))) for j in range(data.n_subchannels) for i in range(0, data.chunk_size, 10)]
        colors_R = [ColorRGBA(*hsva_to_rgba(0.67 - 0.67 * (float(j) / data.n_subchannels), 1.0, np.clip(data.right_channel[i * 40 + j] * 10, 0., 1.))) for j in range(data.n_subchannels) for i in range(0, data.chunk_size, 10)]
        marker = Marker(
            header=Header(frame_id='/map'),
            ns=NODE_NAME,
            id=0,
            type=Marker.POINTS,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.05, 0.05, 0.05),
            color=None,
            lifetime=rospy.Duration(data.chunk_size / data.sample_rate),
            frame_locked=False,
            points=points_L + points_R,
            colors=colors_L + colors_R
            )
        marker_publisher.publish(marker)
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)
    print 'Subscribed'


    r = rospy.Rate(1.)

    while not rospy.is_shutdown():        
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='/map'),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text='Hello world!')
        marker_publisher.publish(marker)
        r.sleep()

if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
