#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
import math
import time

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
    return math.atan(x/h)+2*math.pi/180

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
    return int(round(-0.000130968827*x**2 + 0.189313338*x + 157.439183+0.5))

goal_age = -999
goal_dist = None
goal_angle = None
goal_dist_2 = None
goal_dist_3 = None
balls_dist = []
balls_angle = []

robot_pos = [460/2, 310/2, 0]

def callback_goal(data):
    global goal_dist, goal_angle, goal_dist_2, goal_dist_3, goal_age
    goal_age = time.time()
    goal_rect = data.data
    goal_dist = hinda_kaugust(goal_rect[1]/480.)
    goal_angle = hinda_nurka(goal_rect[0]+goal_rect[2]/2)*180/math.pi
    goal_dist_2 = dist_from_hdata(goal_rect[1])
    goal_dist_3 = dist_from_wdata(goal_rect[2])

def callback_ball(data):
    global balls_angle, balls_dist
    balls_angle = []
    balls_dist = []
    balls = [ (data.data[3*i], data.data[3*i+1], data.data[3*i+2]) for i in range(len(data.data)/3) ]
    balls_angle = []
    for ball in balls:
        balls_angle.append(hinda_nurka(ball[0]))
        d = dist_from_hdata(ball[1])
        if d == 40:
            d = hinda_kaugust(ball[1]/480.)
        balls_dist.append(d)

def get_motor_speeds(ax, ay, omega):
    f1 = ax*0.58 - ay*0.33 + omega*0.33
    f2 = -ax*0.58 - ay*0.33 + omega*0.33
    f3 = ay*0.67 + omega*0.33
    return int(round(f1*128)), int(round(f2*128)), int(round(f3*128))

pub = None
pubmot = None
pubpos = None

def callback_frommotors(data):
    global robot_pos, pubpos
    s1, s2, s3 = 0, 0, 0
    for i in range(len(data.data)):
        if i == 0:
            s1 = data.data[0]
        if i == 1:
            s3 = data.data[1]
        if i == 2:
            s2 = data.data[2]
    sx = s1*0.862069 - s2*0.862069
    sy = - s1*0.5 - s2*0.5 + s3
    so = s1 + s2 + s3
    robot_pos[0] -= sx*0.13523698481710497
    robot_pos[1] -= sy*0.13523698481710497
    robot_pos[2] += so*10000.0*0.0006438629395431943
    
    pubpos.publish(Float32MultiArray(data=[-robot_pos[1], robot_pos[0], robot_pos[2]]))

def __main__():
    global pub, pubmot, pubpos, balls_dist, balls_angle, goal_age, goal_dist_2, goal_dist_3, goal_angle, robot_pos
    rospy.init_node('position_estimater', anonymous=True)
    
    rospy.Subscriber("BallPos", Int32MultiArray, callback_ball)
    rospy.Subscriber("GoalPos", Int32MultiArray, callback_goal)
    rospy.Subscriber("FromMotors", Float32MultiArray, callback_frommotors)
    pub = rospy.Publisher("ToThrower", Int32, queue_size=10)
    pubmot = rospy.Publisher("ToMotors", Int32MultiArray, queue_size=10)
    pubpos = rospy.Publisher("RobotGlobal", Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    state = 0
    tball_dist = None
    tball_ang = None
    goal_counter = 0
    try:
        while not rospy.is_shutdown():
            if state == 5:
                pubmot.publish(Int32MultiArray(data=[0,0,0]))
                print "estimated robot position", robot_pos
            
            if state == 0:
                print "in state 0"
                if balls_dist != []:
                    print "found a ball"
                    tball_dist = balls_dist[0]
                    tball_ang = balls_angle[0]
                    state = 1
                else:
                    print "turning motors on"
                    f1, f2, f3 = get_motor_speeds(0, 0, 0.1)
                    pubmot.publish(Int32MultiArray(data=[f1, f2, f3]))
                
            elif state == 1:
                if tball_dist and tball_dist < 15:
                    print "we're close to the ball"
                    #~ print tball_dist
                    state = 2
                else:
                    print "ball distance:", tball_dist
                    if len(balls_dist) > 0 and balls_dist[0] < 540:
                        tball_dist = balls_dist[0]
                        tball_ang = balls_angle[0]
                    if tball_ang > 1*math.pi/180:
                        print "ball angle is", tball_ang, "greater than 1"
                        omega = -max(min(0.4, tball_ang*180/math.pi/100),0.05)
                    elif tball_ang < -1*math.pi/180:
                        print "ball angle is", tball_ang, "less than 1"
                        omega = max(min(0.4, -tball_ang*180/math.pi/100),0.05)
                    else:
                        omega = 0
                    if omega == 0:
                        vx = max(min(0.4,(tball_dist-20)/20),0.05)
                    else:
                        vx = max(min(0.4,(tball_dist-20)/20),0.05)/3
                    f1, f2, f3 = get_motor_speeds(vx, 0, omega)
                    pubmot.publish(Int32MultiArray(data=[f1, f2, f3]))
            
            elif state == 2:
                print "turning around the ball"
                if time.time() - goal_age > 1:
                    print "otsime varavat"
                    if len(balls_dist) > 0 and balls_dist[0] < 540:
                        tball_dist = balls_dist[0]
                        tball_ang = balls_angle[0]
                    print "ball dist", tball_dist
                    omega = 0
                    vx = 0
                    if tball_ang > 1*math.pi/180:
                        print "poorame vastupaeva"
                        omega = -max(min(0.4, tball_ang*180/math.pi/100),0.05)
                    elif tball_ang < -1*math.pi/180:
                        print "poorame paripaeva"
                        omega = max(min(0.4, -tball_ang*180/math.pi/100),0.05)
                    if tball_dist > 15:
                        print "liigume lahemale"
                        vx = max(min(0.4,(tball_dist-20)/20),0.05)
                    vy = 0.2
                    ms1, ms2, ms3 = get_motor_speeds(vx, vy, omega)
                    pubmot.publish(Int32MultiArray(data=[ms1, ms2, ms3]))
                else:
                    state = 3
            elif state == 3:
                print "varav keskele"
                if len(balls_dist) > 0 and balls_dist[0] < 540:
                    tball_dist = balls_dist[0]
                    tball_ang = balls_angle[0]
                print "ball dist", tball_dist
                vx = 0
                vy = 0
                omega = 0
                if tball_ang > 1*math.pi/180:
                    print "poorame vastupaeva"
                    omega = -max(min(0.4, tball_ang*180/math.pi/30),0.05)
                elif tball_ang < -1*math.pi/180:
                    print "poorame paripaeva"
                    omega = max(min(0.4, -tball_ang*180/math.pi/30),0.05)
                if tball_dist > 15:
                    print "liigume lahemale"
                    vx = max(min(0.4,(tball_dist-20)/20),0.05)
                if goal_angle > 1:
                    vy = -max(min(0.1, goal_angle/100),0.05)
                elif goal_angle < -1:
                    vy = max(min(0.1, -goal_angle/100),0.05)
                elif tball_ang < 1 and tball_ang > -1:
                    state = 4
                    goal_counter = 0
                ms1, ms2, ms3 = get_motor_speeds(vx, vy, omega)
                pubmot.publish(Int32MultiArray(data=[ms1, ms2, ms3]))
                
            elif state == 4:
                if goal_counter > 50:
                    pub.publish(50)
                    state = 0
                else:
                    print "viska!"
                    #~ if goal_exists and goal_angle > 1:
                        #~ omega = -max(min(0.4, goal_angle*180/math.pi/100),0.05)
                    #~ elif goal_exists and goal_angle < -1:
                        #~ omega = max(min(0.4, -goal_angle*180/math.pi/100),0.05)
                    #~ else:
                        #~ omega = 0
                    omega = 0
                    if omega == 0:
                        vx = 0.1
                    else:
                        vx = 0.05
                    f1, f2, f3 = get_motor_speeds(vx, 0, omega)
                    pub.publish(get_throw_speed((goal_dist_2+goal_dist_3)/2))
                    pubmot.publish(Int32MultiArray(data=[f1, f2, f3]))
                    goal_counter += 1
                
                #~ state 
                
                # Otsime palli
            
            #~ if goal_dist_2:
                #~ pub.publish(get_throw_speed((goal_dist_2+goal_dist_3)/2))
                #~ print "goal at angle", goal_angle
            
            #~ print "got a goal at", goal_dist, goal_dist_2, goal_dist_3
            #~ for ball in balls_angle:
                #~ print "got a ball at angle", ball
            
            
            rate.sleep()
    except KeyboardInterrupt:
        pub.publish(50)
        pubmot.publish(Int32MultiArray(data=[0,0,0]))
    except Exception, e:
        print e
        pub.publish(50)
        pubmot.publish(Int32MultiArray(data=[0,0,0]))
    
    pub.publish(50)
    pubmot.publish(Int32MultiArray(data=[0,0,0]))

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pub.publish(50)
        pubmot.publish(Int32MultiArray(data=[0,0,0]))
        pass
    except KeyboardInterrupt:
        pub.publish(50)
        pubmot.publish(Int32MultiArray(data=[0,0,0]))
    except Exception, e:
        print e
        pub.publish(50)
        pubmot.publish(Int32MultiArray(data=[0,0,0]))
