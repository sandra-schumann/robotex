#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import math
import numpy as np
import cv2

balls_pos = [(0,0)]
robot_pos = [310/2, 460/2, 0]

class Vector:
    def __init__(self,x,y,unit):
        self.x = x
        self.y = y
        if unit:
            vlen = (self.x**2+self.y**2)**0.5
            if vlen != 0:
                self.x /= vlen
                self.y /= vlen
    
    def __add__(self, other):
        return Vector(self.x+other.x, self.y+other.y, False)
    
    def __mul__(self, c):
        return Vector(self.x*c, self.y*c, False)


def callback_ball(data):
    global balls_pos
    balls_pos = [ (data.data[2*i], data.data[2*i+1]) for i in range(len(data.data)/2) ]

def callback_towards(data):
    global tp
    if data.data:
        tp = tuple(data.data)

def callback_robot(data):
    global robot_pos
    robot_pos = tuple(data.data)

def get_unit_vector((x,y)):
    vlen = (x**2+y**2)**0.5
    return (x/vlen, y/vlen)

def towards_point((x,y)):
    if robot_pos[0] == x and robot_pos[1] == y:
        return Vector(0,0,False)
    
    dist_to_point = ((robot_pos[0]-x)**2+(robot_pos[1]-y)**2)**0.5
    towards_vec = Vector(x - robot_pos[0],y - robot_pos[1],True)*(20.0/dist_to_point)
    for ballx, bally in balls_pos:
        dist_to_ball = ((robot_pos[0]-ballx)**2+(robot_pos[1]-bally)**2)**0.5
        away_from_ball = Vector(robot_pos[0] - ballx,robot_pos[1] - bally,True)*(6131066257801.0/(dist_to_ball**10))
        towards_vec += away_from_ball
    return towards_vec

def __main__():
    global robot_pos
    rospy.init_node('simulator', anonymous=True)
    
    rospy.Subscriber("BallGlobal", Int32MultiArray, callback_ball)
    rospy.Subscriber("GoTo", Int32MultiArray, callback_towards)
    rospy.Subscriber("RobotGlobal", Float32MultiArray, callback_robot)
    pub = rospy.Publisher("RobotMove", Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        v = towards_point(tp)
        direction = Vector(v.x, v.y, True)
        pub.publish(Float32MultiArray(data=[direction.x, direction.y]))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
