#!/usr/bin/env python

import rospy
import cv2
import numpy as np
#from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2DArray

global pub_detections, pub, flage
flage = False

def video_capture():
    global pub
    #bridge = CvBridge()
    video_capture = cv2.VideoCapture(0)
    
    # rate = rospy.Rate (25)
    __, frame = video_capture.read()
    try:
        #ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

    #except CvBridgeError as e:
    #    print(e)
    except Exception as e: 
        print(e)

    pub.publish(msg) 
    # rate.sleep()
    video_capture.release()
    return frame 

def _results_callback(results):
    global center, size, flage
    detections = results.detections
    center = np.empty((2,len(detections)))
    size = np.empty((2,len(detections)))
    for ii in range (len(detections)):
        center[0,ii] = detections[ii].bbox.center.x
        center[1,ii] = detections[ii].bbox.center.y
        size[0,ii] = detections[ii].bbox.size_x
        size[1,ii] = detections[ii].bbox.size_y
    
    if detections != []:
        flage = True

def video_yolo_classification_pub(frame):
    global center, size, flage, pub_detections
    
    try:
        # while True:
            # window_name = 'Image'
        if flage == True:
            flage = False 
            for ii in range (len(center[0,:])):
                left = int((center[0,ii] - size[0,ii] / 2.0))
                right = int((center[0,ii] + size[0,ii] / 2.0))
                top = int((center[1,ii] - size[1,ii] / 2.0))
                bottom = int((center[1,ii] + size[1,ii] / 2.0))
                start_point = (left, top); end_point = (right, bottom); color = (255, 0, 0); thickness = 2
                image = cv2.rectangle(frame, start_point, end_point, color, thickness) 
                circle_thickness = -1; radius = 15; circle_center = (int(center[0,ii]),int(center[1,ii]))
                image = cv2.circle(frame, circle_center, radius, color, circle_thickness)

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            pub_detections.publish(msg)

                # cv2.imshow(window_name, image)
                # cv2.waitKey(1)
                # #cv2.destroyAllWindows()
                # break
        else:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            pub_detections.publish(msg) 
    except: 
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        pub_detections.publish(msg) 
        # window_name = 'Image'
        # while True:
        #     # cv2.imshow(window_name, frame)
        #     # cv2.waitKey(1)
        #     # #cv2.destroyAllWindows()
        #     break


rospy.init_node('video_handler', anonymous=True)
rospy.Subscriber('/yolo/results', Detection2DArray, _results_callback)
pub_detections = rospy.Publisher('/yolo/results/compressed/', CompressedImage, queue_size=2)
pub = rospy.Publisher('/image_raw/yolo/compressed', CompressedImage, queue_size=2)

while not rospy.is_shutdown():
    frame = video_capture()
    try:
        video_yolo_classification_pub(frame)
    except: pass #Exception as e: 
        #print(e)
