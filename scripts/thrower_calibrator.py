#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray, Int32
import time
import math

goal_angle = -999
goal_dist = -999
dist_list = []

with open("calibration_data_height.txt", 'r') as fin:
    height_t = [ (int(line[0]), int(line[1])) for line in [ line.split() for line in fin.read().split('\n') if line ] ]
with open("calibration_data_width.txt", 'r') as fin:
    width_t = [ (int(line[0]), int(line[1])) for line in [ line.split() for line in fin.read().split('\n') if line ] ]

def dist_from_hdata(x):
    for i in range(len(height_t)):
        d,h = height_t[i]
        if h > x:
            if i == 0:
                return d
            prevd, prevx = height_t[i-1]
            if prevx == x:
                return (d + prevd)/2
            return (x-prevx)*(d-prevd)/(h-prevx) + prevd
    return height_t[-1][0]+1

def dist_from_wdata(x):
    for i in range(len(width_t)):
        d,w = width_t[i]
        if w < x:
            if i == 0:
                return d
            prevd, prevw = width_t[i-1]
            if prevw == x:
                return (d + prevd)/2
            return (x-prevw)*(d-prevd)/(w-prevw) + prevd
    return width_t[-1][0]+1

def callback(data):
    global goal_angle, goal_dist
    goal_rect = data.data
    goal_dist = (dist_from_wdata(goal_rect[2]) + dist_from_hdata(goal_rect[1]))/2
    goal_angle = hinda_nurka(goal_rect[0]+goal_rect[2]/2)
    
def hinda_nurka(x):
    x -= 320
    ra = 80.0*math.pi/180
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

def __main__():
    global goal_angle, goal_dist
    rospy.init_node('thrower_calibrator', anonymous=True)

    rospy.Subscriber("GoalPos", Int32MultiArray, callback)
    pub = rospy.Publisher("ToThrower", Int32, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if goal_angle*180/math.pi > 1:
            print "angle too big"
        elif goal_angle*180/math.pi < -1:
            print "angle too small"
        else:
            s = int(raw_input("Enter desired speed: "))
            pub.publish(s)
            rate.sleep()
            sisse = int(raw_input("Sisse? "))
            pub.publish(50)
            if sisse == 1:
                dist_list.append((goal_dist, s))
            elif sisse == -1:
                break
        
        rate.sleep()
    
    fout = open("throw_calibration_data.txt", 'w')
    for x,y in sorted(dist_list):
        fout.write(str(x) + ' ' + str(y) + '\n')
    fout.close()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
