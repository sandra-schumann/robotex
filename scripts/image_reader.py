#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

rospy.init_node('VideoPublisher', anonymous=True)

rawimage = rospy.Publisher('VideoRaw', Image, queue_size=10)

cap = cv2.VideoCapture(0)

#~ while cap.isOpened():
while not rospy.is_shutdown():
    _, frame = cap.read()
    
    msg_frame = CvBridge().cv2_to_imgmsg(frame)
    
    #~ rospy.loginfo(msg_frame)
    rawimage.publish(msg_frame)

    time.sleep(0.01)
