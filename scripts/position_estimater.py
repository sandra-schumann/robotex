#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import math

def hinda_kaugust(b_frac):
    h = 30.525
    a = 26.0 #22.0
    phi = 0.8901179
    
    alpha = math.atan(float(a)/h)
    f = math.sin(phi)*(h**2+a**2)**0.5/math.sin(math.pi/2-phi/2)
    b = f*b_frac
    d = (h**2 + a**2 + b**2 - 2*(h**2 + a**2)**0.5*b*math.cos(math.pi/2-phi/2))**0.5
    beta = math.asin(b*math.sin(math.pi/2-phi/2)/d)
    x = h*math.tan(alpha+beta)
    x_corr = x #-1.061051392*(13.284393419-x) #-1.26582*(11.85-x)
    return x_corr

def hinda_nurka(x):
    ra = 65.0*math.pi/180
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

goal_dist = None
goal_angle = None
balls_dist = []
balls_angle = []

def callback_goal(data):
    global goal_dist, goal_angle
    goal_rect = data.data
    goal_dist = hinda_kaugust(goal_rect[1]/480.)
    goal_angle = hinda_nurka(goal_rect[0]+goal_rect[2]/2-320)*180/math.pi

def callback_ball(data):
    balls = [ (data.data[3*i], data.data[3*i+1], data.data[3*i+2]) for i in range(len(data.data)/3) ]
    #~ for ball in balls:
        #~ balls_dist = 

def __main__():
    rospy.init_node('position_estimater', anonymous=True)
    
    rospy.Subscriber("BallPos", Int32MultiArray, callback_ball)
    rospy.Subscriber("GoalPos", Int32MultiArray, callback_goal)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print "got a goal at", goal_dist, goal_angle
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
