#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from random import randint

def talker():
    pub = rospy.Publisher("ToThrower", Int32, queue_size=10)
    rospy.init_node('thrower_tester', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        s = Int32(raw_input("Enter desired speed: "))
        rospy.loginfo(s)
        pub.publish(s)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
