#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray
import time

goal_dist = -1
goal_age = -999

dist_list = []

def callback(data):
    global goal_dist, goal_age
    goal_dist = data.data[1]
    goal_age = time.time()
    
def __main__():
    rospy.init_node('distance_calibrator', anonymous=True)

    rospy.Subscriber("GoalPos", Int32MultiArray, callback)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if time.time() - goal_age > 1:
            print "no goal"
        else:
            print "goal at pixel", goal_dist
            real_dist = int(raw_input("What is the real distance to the goal right now? "))
            if real_dist == -1:
                break
            dist_list.append((real_dist, goal_dist))
        
        rate.sleep()
    
    fout = open("calibration_data.txt", 'w')
    for x,y in sorted(dist_list):
        fout.write(str(x) + ' ' + str(y) + '\n')
    fout.close()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
