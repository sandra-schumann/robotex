#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

#~ cap = cv2.VideoCapture(0)

print cv2.__version__

fin = open("scripts/ball_color.txt", 'r')
t = [ line.split(' ') for line in fin.read().split('\n') ]
lower_ball = np.array([int(t[0][0]),int(t[0][1]),int(t[0][2])])
upper_ball = np.array([int(t[1][0]),int(t[1][1]),int(t[1][2])])

minradius = 10

frame = None

ballpos = None

def callback(data):
    global frame
    frame = CvBridge().imgmsg_to_cv2(data)
    detect_ball()

def detect_ball():
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.medianBlur(frame,1)
    
    #~ # Define range of green color in HSV
    lower_ball = np.array([45,58,62])
    upper_ball = np.array([75,174,255])
        
    # Threshold the HSV image to get only appropriate colors
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    # Bitwise-AND mask and original image
    ball_res = cv2.bitwise_and(frame, frame, mask=ball_mask)
    
    balls = []
    if cv2.countNonZero(ball_mask) != 0:
        _,contours,_ = cv2.findContours(ball_mask, 1, 2)
        if contours:
            contours.sort(key = lambda x : cv2.minEnclosingCircle(x)[1])
            contours.reverse()
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            if radius < 5:
                break
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(frame,center,radius,(0,255,0),2)
            balls += [int(x), int(y), radius]
    
    if len(balls) > 0:
        return (balls[0][0],balls[0][1])
    return (0,0)
    
    #~ return ball_midpoint
    ballpos = balls
    #~ print "put a ball at", ballpos
    
    #~ cv2.imshow('Camera image',frame)

def talk_ball_pos():
    global ballpos
    rospy.init_node('ball_detector', anonymous=True)
    pub = rospy.Publisher("ball_pos", Int32MultiArray, queue_size=10)
    rospy.Subscriber("VideoRaw", Image, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if ballpos == None:
            pass
        else:
            ball_positions = Int32MultiArray(data=ballpos)
            rospy.loginfo(ball_positions)
            pub.publish(ball_positions)
        rate.sleep()

def __main__():
    talk_ball_pos()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
