#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32MultiArray
from random import randint

def talker():
    pub = rospy.Publisher("ToMotors", Int32MultiArray, queue_size=10)
    rospy.init_node('motor_tester', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    direction = 1
    while not rospy.is_shutdown():
        dirs = Int32MultiArray(data=[10*direction, 10*direction, 10*direction])
        direction = direction*(-1)
        rospy.loginfo(dirs)
        pub.publish(dirs)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
