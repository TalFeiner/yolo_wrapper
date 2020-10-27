#!/usr/bin/env python

import rospy
import cv2
import numpy as np
#from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge, CvBridgeError

global out, c, frame_num  
frame_num = 6000
c = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')

def out_fun(shape):
    global out
    out = cv2.VideoWriter('corridor_video.avi',fourcc, 20.0, (shape[1], shape[0]))

def _shutdown():
    global out, c
    out.release()
    print c, " frames has been captured"
    print "shutdown"


def _Img_callback(ros_data):
    global out, c

    np_arr = np.fromstring(ros_data.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if (c == 0):
        frame_shape = np.shape(frame)
        out_fun(frame_shape)

    #frame = cv2.flip(frame,0)
    #cv2.imwrite("table_"+str(c)+'.jpg',frame)
    out.write(frame)
    rospy.sleep(0.1)
    c = 1+c

    print "The number amount of captured frames: ", c
    if (c == frame_num):
        print "Video end "
        rospy.signal_shutdown("Video end")


rospy.init_node('video_handler', anonymous=True)
rospy.on_shutdown(_shutdown)

#bridge = CvBridge()

rospy.Subscriber("/kinect2/qhd/image_color_rect/compressed",
                         CompressedImage, _Img_callback, queue_size=2)
rospy.wait_for_message("/kinect2/qhd/image_color_rect/compressed", CompressedImage)

rospy.spin()

