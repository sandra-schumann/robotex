#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Int32
import math

distance_corrector = [46.217, 50.82, 57.0, 63.51, 68.68, 73.76, 79.03, 83.42, 88.82, 93.59, 96.8, 101.64, 103.85, 107.71, 111.85, 116.27, 121.02, 126.12, 128.28, 130.5, 132.79, 136.36, 137.6, 142.73, 142.73, 144.07, 148.22, 146.81, 149.66, 152.6, 154.12, 162.12, 162.12, 160.46, 165.54, 163.8, 167.3]

with open("calibration_data_height.txt", 'r') as fin:
    height_t = [ (int(line[0]), int(line[1])) for line in [ line.split() for line in fin.read().split('\n') if line ] ]
with open("calibration_data_width.txt", 'r') as fin:
    width_t = [ (int(line[0]), int(line[1])) for line in [ line.split() for line in fin.read().split('\n') if line ] ]

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
    x_corr = 1.23392*x-23.698 #-1.061051392*(13.284393419-x) #-1.26582*(11.85-x)
    return x_corr

def hinda_nurka(x):
    x -= 320
    ra = 80.0*math.pi/180
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

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

def get_throw_speed(x):
    return int(round(-0.000130968827*x**2 + 0.189313338*x + 157.439183))

goal_dist = None
goal_angle = None
goal_dist_2 = None
goal_dist_3 = None
balls_dist = []
balls_angle = []

def callback_goal(data):
    global goal_dist, goal_angle, goal_dist_2, goal_dist_3
    goal_rect = data.data
    goal_dist = hinda_kaugust(goal_rect[1]/480.)
    goal_angle = hinda_nurka(goal_rect[0]+goal_rect[2]/2)*180/math.pi
    goal_dist_2 = dist_from_hdata(goal_rect[1])
    goal_dist_3 = dist_from_wdata(goal_rect[2])

def callback_ball(data):
    global balls_angle
    balls = [ (data.data[3*i], data.data[3*i+1], data.data[3*i+2]) for i in range(len(data.data)/3) ]
    balls_angle = []
    for ball in balls:
        balls_angle.append(hinda_nurka(ball[0]))

def __main__():
    rospy.init_node('position_estimater', anonymous=True)
    
    rospy.Subscriber("BallPos", Int32MultiArray, callback_ball)
    rospy.Subscriber("GoalPos", Int32MultiArray, callback_goal)
    pub = rospy.Publisher("ToThrower", Int32, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        if goal_dist_2:
            pub.publish(get_throw_speed((goal_dist_2+goal_dist_3)/2))
            print "goal at angle", goal_angle
        
        #~ print "got a goal at", goal_dist, goal_dist_2, goal_dist_3
        #~ for ball in balls_angle:
            #~ print "got a ball at angle", ball
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
