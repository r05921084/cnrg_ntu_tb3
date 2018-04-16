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
CHUNK_SIZE = 1024
N_SUBCHANNELS = 40


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

    pos = np.zeros([2, CHUNK_SIZE, N_SUBCHANNELS, 3])
    pos[:, :, :, 0] = np.arange(CHUNK_SIZE).reshape([-1, 1]) * 0.01 + np.zeros(N_SUBCHANNELS) - CHUNK_SIZE * 0.005
    pos[:, :, :, 1] = np.arange(N_SUBCHANNELS) * 0.1 + 0.5
    pos[1, :, :, 1] += (- 1.0 - N_SUBCHANNELS * 0.1)

    clr = np.zeros([2, CHUNK_SIZE, N_SUBCHANNELS, 4])
    for j in range(N_SUBCHANNELS):
        clr[:, :, j, :] = np.array(hsva_to_rgba(0.67 - 0.67 * j / N_SUBCHANNELS, 1.0, 1.0, 1.0))
    
    def ani_cb(data):
        pos[0, :, :, 2] = np.array(data.left_channel).reshape( [CHUNK_SIZE, N_SUBCHANNELS]) * 10
        pos[1, :, :, 2] = np.array(data.right_channel).reshape([CHUNK_SIZE, N_SUBCHANNELS]) * 10
        
        points_L = [Point(*pos[0, i, j]) for j in range(N_SUBCHANNELS) for i in range(0, CHUNK_SIZE, 10)]
        points_R = [Point(*pos[1, i, j]) for j in range(N_SUBCHANNELS) for i in range(0, CHUNK_SIZE, 10)]
        colors_L = [ColorRGBA(*clr[0, i, j]) for j in range(N_SUBCHANNELS) for i in range(0, CHUNK_SIZE, 10)]
        colors_R = [ColorRGBA(*clr[1, i, j]) for j in range(N_SUBCHANNELS) for i in range(0, CHUNK_SIZE, 10)]
        
        marker = Marker(
            header=Header(frame_id='/map'),
            ns=NODE_NAME,
            id=1,
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
                header=Header(frame_id='/map'),
                ns=NODE_NAME,
                id=0,
                type=Marker.TEXT_VIEW_FACING,                
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(0., 0., 1.), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),                
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text=NODE_NAME)
        marker_publisher.publish(marker)
        r.sleep()

if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
