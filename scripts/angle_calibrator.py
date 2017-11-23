#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import time
import math

balls_angle = []
dist_list = []

def callback(data):
    global balls_angle
    balls = [ (data.data[3*i], data.data[3*i+1], data.data[3*i+2]) for i in range(len(data.data)/3) ]
    balls_angle = []
    for ball in balls:
        balls_angle.append(ball[0])
    
def hinda_nurka(x):
    x -= 320
    ra = 80.0*math.pi/180
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

def __main__():
    rospy.init_node('distance_calibrator', anonymous=True)

    rospy.Subscriber("BallPos", Int32MultiArray, callback)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if balls_angle:
            print "ball at angle", balls_angle[0], hinda_nurka(balls_angle[0])*180/math.pi
            real_dist = int(raw_input("What is the real distance right now? "))
            if real_dist == -1:
                break
            print "angle should be", math.atan(real_dist/50.0)*180/math.pi
        
        rate.sleep()
    
    #~ fout = open("angle_calibration_data.txt", 'w')
    #~ for x,y in sorted(dist_list):
        #~ fout.write(str(x) + ' ' + str(y) + '\n')
    #~ fout.close()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
