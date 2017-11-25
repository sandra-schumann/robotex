#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import aruco

cap = cv2.VideoCapture(0)

print cv2.__version__

with open("scripts/ball_color.txt", 'r') as fin:
    t = [ line.split(' ') for line in fin.read().split('\n') ]
    lower_ball = np.array([int(t[0][0]),int(t[0][1]),int(t[0][2])])
    upper_ball = np.array([int(t[1][0]),int(t[1][1]),int(t[1][2])])

with open("scripts/goal_color.txt", 'r') as fin:
    t = [ line.split(' ') for line in fin.read().split('\n') ]
    lower_goal = np.array([int(t[0][0]),int(t[0][1]),int(t[0][2])])
    upper_goal = np.array([int(t[1][0]),int(t[1][1]),int(t[1][2])])

with open("scripts/field_color.txt", 'r') as fin:
    t = [ line.split(' ') for line in fin.read().split('\n') ]
    lower_field = np.array([int(t[0][0]),int(t[0][1]),int(t[0][2])])
    upper_field = np.array([int(t[1][0]),int(t[1][1]),int(t[1][2])])

minradius = 2

def detect_ball(hsv, frame, linepoints):
    #~ # Define range of green color in HSV
    #~ lower_ball = np.array([45,58,62])
    #~ upper_ball = np.array([75,174,255])
    
    # Threshold the HSV image to get only appropriate colors
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    # Bitwise-AND mask and original image
    ball_res = cv2.bitwise_and(hsv, hsv, mask=ball_mask)
    
    balls = []
    if cv2.countNonZero(ball_mask) != 0:
        _,contours,_ = cv2.findContours(ball_mask, 1, 2)
        if contours:
            contours.sort(key = lambda x : cv2.minEnclosingCircle(x)[1])
            contours.reverse()
        for cnt in contours:
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            outside = False
            for ((x1,y1),(x2,y2)) in linepoints:
                vecline = np.array([(x2-x1), (y2-y1)])
                vecmidpoint = np.array([(320-x1), (0-y1)])
                vecball = np.array([(x-x1), (y-y1)])
                if not((np.cross(vecline, vecmidpoint) > 0 and np.cross(vecline, vecball) > 0) or (np.cross(vecline, vecmidpoint) < 0 and np.cross(vecline, vecball) < 0)):
                    outside = True
                    break
            if outside:
                continue
            if radius < minradius:
                break
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(frame,center,radius,(0,255,0),2)
            balls += [int(x), int(y), radius]
    
    return balls
    
    #~ cv2.imshow('Camera image',hsv)
    #~ k = cv2.waitKey(5) & 0xFF

def detect_goal(frame):
    detector = aruco.MarkerDetector()
    markers = detector.detect(frame)
    
    m10x, m10y, m10w, m10h, m10d = -1, -1, -1, -1, -1
    m11x, m11y, m11w, m11h, m11d = -1, -1, -1, -1, -1
    for marker in markers:
        if marker.id == 10:
            m10x = int(round((marker[1][0]+marker[2][0])/2))
            m10y = int(round((marker[2][1]+marker[3][1])/2))
            m10w = int(round((marker[0][0]+marker[3][0])/2-m10x))
            m10h = int(round((marker[0][1]+marker[1][1])/2-m10y))
            m10d = int(round(abs(marker[2][1]-marker[3][1])))
        if marker.id == 11:
            m11x = int(round((marker[1][0]+marker[2][0])/2))
            m11y = int(round((marker[2][1]+marker[3][1])/2))
            m11w = int(round((marker[0][0]+marker[3][0])/2-m11x))
            m11h = int(round((marker[0][1]+marker[1][1])/2-m11y))
            m11d = int(round(abs(marker[2][1]-marker[3][1])))
        else:
            print "seeing marker", marker.id
    
    goal_rect = [-1,-1,-1,-1]
    
    if m10x != -1 and m10y != -1 and m11x != -1 and m11y != -1:
        goal_rect[0] = m11x+m11w
        goal_rect[1] = m11y
        goal_rect[2] = m10x-m11x-m11w
        goal_rect[3] = (m10h+m11h)/2
        x, y, w, h = goal_rect[0], goal_rect[1], goal_rect[2], goal_rect[3]
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    elif m10x != -1 and m10y != -1:
        goal_rect[0] = int(round(m10x-m10w*0.75))
        goal_rect[1] = int(round(m10y+0.75*m10d))
        goal_rect[2] = int(round(m10w*0.75))
        goal_rect[3] = m10h
        x, y, w, h = goal_rect[0], goal_rect[1], goal_rect[2], goal_rect[3]
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    elif m11x != -1 and m11y != -1:
        goal_rect[0] = m11x+m11w
        goal_rect[1] = int(round(m11y+0.75*m11d))
        goal_rect[2] = int(round(m11w*0.75))
        goal_rect[3] = m11h
        x, y, w, h = goal_rect[0], goal_rect[1], goal_rect[2], goal_rect[3]
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    
    #~ cv2.imshow("",frame)
    #~ k = cv2.waitKey(5) & 0xFF
    
    return goal_rect

def get_lines(hsv,frame):
    black_line_low = np.array([0, 184, 0])
    black_line_high = np.array([180, 255, 100])
    
    field_mask = cv2.inRange(hsv, lower_field, upper_field)
    black_mask = cv2.inRange(hsv, black_line_low, black_line_high)
    
    hsv = cv2.bitwise_and(hsv, hsv, mask=black_mask)
    #~ cv2.imshow("hsv",hsv)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[:400,:640]
    edges = cv2.Canny(black_mask,200,500)
    lines = cv2.HoughLines(edges,1,np.pi/180,200)
    
    linepoints = []
    
    try:
        for line in lines:
            for rho,theta in line:
                #~ print rho, theta
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                linepoints.append(((x1,y1),(x2,y2)))
                cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
    except:
        pass
    
    return linepoints

def __main__():
    print "started image reading"
    frame = None
    balls = []
    goalpos = [-1, -1, -1, -1]
    
    rospy.init_node('image_reader', anonymous=True)
    ballpub = rospy.Publisher("BallPos", Int32MultiArray, queue_size=10)
    goalpub = rospy.Publisher("GoalPos", Int32MultiArray, queue_size=10)
    linepub = rospy.Publisher("LinePos", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    
    while not rospy.is_shutdown():
        _, frame = cap.read()
        # Convert BGR to HSV
        #~ frame = cv2.medianBlur(frame,1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:400,:640]
        
        linepoints = get_lines(hsv,frame)
        
        balls = detect_ball(hsv, frame, linepoints)
        goalpos = detect_goal(frame)
        
        if balls != []:
            ball_positions = Int32MultiArray(data=balls)
            #~ rospy.loginfo(ball_positions)
            ballpub.publish(ball_positions)
        
        if sum(goalpos) != -4:
            goal_positions = Int32MultiArray(data=goalpos)
            #~ rospy.loginfo(goal_positions)
            goalpub.publish(goal_positions)
        
        #~ try:
            #~ linepub_data = []
            #~ for line in lines:
                #~ print line
                #~ linepub_data.append(line[0][0])
                #~ linepub_data.append(line[0][1])
            #~ if linepub_data:
                #~ linepub_data = Float32MultiArray(data=linepub_data)
                #~ rospy.loginfo(linepub_data)
                #~ linepub.publish(linepub_data)
        #~ except Exception, e:
            #~ print e
        
        #~ cv2.imshow("",frame)
        #~ k = cv2.waitKey(5) & 0xFF
        #~ if k == 27:
            #~ break
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        if cap:
            cap.close()
    except KeyboardInterrupt:
        if cap:
            cap.close()
