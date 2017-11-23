#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import math
import numpy as np
import cv2

balls_pos = [(0,0),(-75,-100),(-75,100),(75,-100),(75,100)]
robot_pos = (310.0/2, 460.0/2, 0)
tp = (310.0/2, 460.0/2)
goal1_pos = (0.0,-238.0)
goal2_pos = (0.0, 238.0)

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

def angle_to_goal((goal_x, goal_y)):
    to_goal_x = goal_x - robot_pos[0]
    to_goal_y = robot_pos[1] - goal_y
    if to_goal_y == 0:
        if to_goal_x > 0:
            return 900
        elif to_goal_x < 0:
            return 1800
        return 0
    ans = math.atan(to_goal_x/to_goal_y)*1800/math.pi
    if to_goal_y < 0:
        ans += 1800
    return ans

def dist_points((x1,y1),(x2,y2)):
    return ((x1-x2)**2+(y1-y2)**2)**0.5

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
    towards_vec = Vector(x - robot_pos[0],y - robot_pos[1],True)*(max(20.0/dist_to_point, 1))
    for ballx, bally in balls_pos:
        dist_to_ball = ((robot_pos[0]-ballx)**2+(robot_pos[1]-bally)**2)**0.5
        away_from_ball = Vector(robot_pos[0] - ballx,robot_pos[1] - bally,True)*(6131066257801.0/(dist_to_ball**10))
        towards_vec += away_from_ball
    dist_from_basket1 = ((robot_pos[0] - goal1_pos[0])**2 + (robot_pos[1] - goal1_pos[1])**2)**0.5
    away_goal1 = Vector(robot_pos[0] - goal1_pos[0], robot_pos[0] - goal1_pos[1], True)*(3010936384.0/(dist_from_basket1**6))
    towards_vec += away_goal1
    dist_from_basket2 = ((robot_pos[0] - goal2_pos[0])**2 + (robot_pos[1] - goal2_pos[1])**2)**0.5
    away_goal2 = Vector(robot_pos[0] - goal2_pos[0], robot_pos[0] - goal2_pos[1], True)*(3010936384.0/(dist_from_basket2**6))
    towards_vec += away_goal2
    return towards_vec

def __main__():
    global robot_pos, balls_pos
    rospy.init_node('simulator', anonymous=True)
    
    rospy.Subscriber("BallGlobal", Int32MultiArray, callback_ball)
    rospy.Subscriber("GoTo", Int32MultiArray, callback_towards)
    rospy.Subscriber("RobotGlobal", Float32MultiArray, callback_robot)
    pub = rospy.Publisher("RobotMove", Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(50)
    
    state = 0
    target_ball = None
    target_ball_index = None
    
    while not rospy.is_shutdown():
        if tp == (-1,-1):
            if state == 0:
                # Looking for nearest ball
                mindist = 100000.0
                for i, ball in enumerate(balls_pos):
                    dist_ball = dist_points((robot_pos[0], robot_pos[1]),ball)
                    if dist_ball < mindist:
                        mindist = dist_ball
                        target_ball = ball
                        target_ball_index = i
                if target_ball != None:
                    # Should turn and look for balls if no target
                    ball = None
                    state = 1
            elif state == 1:
                # Find target position
                target_vec = Vector(target_ball[0] - goal1_pos[0], target_ball[1] - goal1_pos[1], True)*30
                target_pos = (target_ball[0] + target_vec.x, target_ball[1] + target_vec.y)
                if dist_points(target_pos, (robot_pos[0], robot_pos[1])) < 2:
                    state = 2
                else:
                    v = towards_point(target_pos)
                    direction = Vector(v.x, v.y, True)
                    pub.publish(Float32MultiArray(data=[direction.x, direction.y]))
            elif state == 2:
                if dist_points(target_ball, (robot_pos[0], robot_pos[1])) < 2:
                    target_ball = None
                    balls_pos = balls_pos[:target_ball_index] + balls_pos[target_ball_index+1:]
                    state = 0
                else:
                    throw_ball_vec = Vector(target_ball[0] - robot_pos[0], target_ball[1] - robot_pos[1], True)
                    pub.publish(Float32MultiArray(data=[throw_ball_vec.x, throw_ball_vec.y]))
            
        else:
            state = 0
            target_ball == None
            v = towards_point(tp)
            direction = Vector(v.x, v.y, True)
            pub.publish(Float32MultiArray(data=[direction.x, direction.y]))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
