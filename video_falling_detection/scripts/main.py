#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from detection_model import model
from matplotlib import axis

bridge = CvBridge()

def callback(data):
    global pic
    # rospy.loginfo(bridge.imgmsg_to_cv2(data, 'bgr8'))
    pic = bridge.imgmsg_to_cv2(data, 'bgr8')
    # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

#def model():
#    global pic
#
#    if pic.mean()<30:
#        rospy.loginfo('ABNORMALL!!!!!!!!!!!!!!')
#    else:
#        rospy.loginfo('========NORMALL========')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True, disable_signals=True)
    #rospy.init_node('household_situation', anonymous=True)

    pub = rospy.Publisher('household_situation', String, queue_size=10)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    counter = 0
    num_of_frames = 50
    image_buffer = np.zeros((num_of_frames,480,640,3))
    index = np.arange(num_of_frames)  # for feed data to model with right order.
    data_ready = False

    while True:
    
        if len(pic) == 0:
            #print("NO IMAGE !!!!!!!!!!!!!!!")
            pub.publish("NO IMAGE !!!!!!!!!!!!!!!")

        else:
            # model(pic)
            image_buffer[counter % num_of_frames]=pic
            counter += 1
        
        if not data_ready:
            buffer_condition = (np.count_nonzero(image_buffer, 1).sum(axis=1) != 0).sum()  #check every frame having image
            if buffer_condition == 150:
                data_ready = True
                print('data is ready for model')

        if data_ready:
            print((index+counter)%num_of_frames)
            model(image_buffer[(index+counter)%num_of_frames], pub)
        else:
            pub.publish("buffer not full[{}] !!!!!!!!!!!!!!!".format(buffer_condition))

        
    spin()
    # simply keeps python from exiting until this node is stopped
    # rospy.spin()

pic = []

if __name__ == '__main__':
    listener()
