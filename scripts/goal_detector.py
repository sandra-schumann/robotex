#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

#~ cap = cv2.VideoCapture(0)

fin = open("scripts/goal_color.txt", 'r')
t = [ line.split(' ') for line in fin.read().split('\n') ]
lower_goal = np.array([int(t[0][0]),int(t[0][1]),int(t[0][2])])
upper_goal = np.array([int(t[1][0]),int(t[1][1]),int(t[1][2])])

frame = None

def callback(data):
    global frame
    frame = CvBridge().imgmsg_to_cv2(data)

def detect_goal():
    global frame
    # Take each frame
    #~ _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.medianBlur(frame,5)
    
    #~ # Define range of blue color in HSV
    #~ lower_goal = np.array([99,133,75])
    #~ upper_goal = np.array([107,255,255])
        
    # Threshold the HSV image to get only appropriate colors
    goal_mask = cv2.inRange(hsv, lower_goal, upper_goal)
    # Bitwise-AND mask and original image
    goal_res = cv2.cvtColor(cv2.bitwise_and(frame, frame, mask=goal_mask), cv2.COLOR_BGR2GRAY)
    
    if cv2.countNonZero(goal_mask) != 0:
        ret,thresh = cv2.threshold(goal_mask,127,255,0)
        _,contours,_ = cv2.findContours(thresh, 1, 2)
        contours.sort(key = lambda cnt : cv2.boundingRect(cnt)[2]*cv2.boundingRect(cnt)[3])
        if len(contours) > 0:
            cnt = contours[len(contours)-1]
            goal_rect = cv2.boundingRect(cnt)
            x,y,w,h = goal_rect
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        else:
            goal_rect = [-1,-1,-1,-1]
    else:
        goal_rect = [-1,-1,-1,-1]
    
    return goal_rect
    
    #~ cv2.imshow('Camera image',frame)

def talk_goal_pos():
    pub = rospy.Publisher("goal_pos", Int32MultiArray, queue_size=10)
    raw = rospy.Subscriber("VideoRaw", Image, callback)
    rospy.init_node('goal_detector', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goalpos = detect_goal()
        goal_positions = Int32MultiArray(data=goalpos)
        rospy.loginfo(goal_positions)
        pub.publish(goal_positions)
        rate.sleep()

def __main__():
    talk_goal_pos()
    #~ while 1:
        #~ detect_goal()
        #~ k = cv2.waitKey(5) & 0xFF
        #~ if k == 27:
            #~ break

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
