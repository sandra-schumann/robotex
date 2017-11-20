#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import math
import numpy as np
import cv2

balls_pos = [(0,0),(-75,-100),(-75,100),(75,-100),(75,100)]
robot_pos = [310/2, 460/2, 0]

def callback_ball(data):
    global balls_pos
    balls_pos = [ (data.data[2*i], data.data[2*i+1]) for i in range(len(data.data)/2) ]

def angle_to_goal():
    goal_x = 0.0
    goal_y = -238.0
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

def callback_robotmove(data):
    global robot_pos
    move_x, move_y = data.data
    robot_pos[0] += move_x*2
    robot_pos[1] += move_y*2
    robot_pos[2] = angle_to_goal()

def __main__():
    global balls_pos
    
    rospy.init_node('simulator', anonymous=True)
    
    rospy.Subscriber("BallGlobal", Int32MultiArray, callback_ball)
    pub = rospy.Publisher("RobotGlobal", Float32MultiArray, queue_size=10)
    rospy.Subscriber("RobotMove", Float32MultiArray, callback_robotmove)
    
    img = np.zeros((610,400,3), np.uint8)
    
    cv2.rectangle(img,(0,0),(400,610),(0,127,255),-1)
    cv2.rectangle(img,(47,77),(353,533),(0,0,0),3)
    cv2.rectangle(img,(49,79),(351,531),(255,255,255),2)
    cv2.line(img,(49,305),(351,305),(255,255,255),2)
    cv2.circle(img,(200,305),40,(255,255,255),2)
    cv2.circle(img,(200,67),8,(176,124,0),-1)
    cv2.circle(img,(200,543),8,(119,64,188),-1)
    
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        pub.publish(Float32MultiArray(data=robot_pos))
        
        rob_pos = tuple([ int(round(x)) for x in robot_pos ])
        
        # Check for robot hitting the balls
        i = 0
        while i < len(balls_pos):
            ball = balls_pos[i]
            if dist_points(ball, (robot_pos[0], robot_pos[1])) < 17:
                balls_pos = balls_pos[:i] + balls_pos[i+1:]
                i -= 1
            i += 1
        
        robot_mask = cv2.bitwise_not(np.zeros((610,400,3), np.uint8))
        cv2.circle(robot_mask,(rob_pos[0]+200,rob_pos[1]+305),17,(0,0,0),-1)
        cv2.line(robot_mask,(rob_pos[0]+200,rob_pos[1]+305),
        (int(round(rob_pos[0]+200+17*math.sin(rob_pos[2]*math.pi/1800.))),int(round(rob_pos[1]+305-17*math.cos(rob_pos[2]*math.pi/1800.)))),(255,255,255),3)
        
        ball_mask = np.zeros((610,400,3), np.uint8)
        remove_nonballs = cv2.bitwise_not(np.zeros((610,400,3), np.uint8))
        for ball in balls_pos:
            cv2.circle(ball_mask,(ball[0]+200,ball[1]+305),4,(0,255,0),-1)
            cv2.circle(remove_nonballs,(ball[0]+200,ball[1]+305),4,(0,0,0),-1)
        
        cv2.imshow("Simulator", cv2.add(cv2.bitwise_and(cv2.bitwise_and(img,robot_mask),remove_nonballs),ball_mask))
        
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
