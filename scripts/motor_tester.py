#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32MultiArray
from random import randint

def get_motor_speeds(ax, ay, omega):
    f1 = ax*0.58 - ay*0.33 + omega*0.33
    f2 = -ax*0.58 - ay*0.33 + omega*0.33
    f3 = ay*0.67 + omega*0.33
    return int(round(f1*128)), int(round(f2*128)), int(round(f3*128))

def talker():
    pub = rospy.Publisher("ToMotors", Int32MultiArray, queue_size=10)
    rospy.init_node('motor_tester', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    direction = 1
    while not rospy.is_shutdown():
        goright = get_motor_speeds(0,0.1,0)
        dirs = Int32MultiArray(data=goright)
        direction = direction*(-1)
        rospy.loginfo(dirs)
        pub.publish(dirs)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
