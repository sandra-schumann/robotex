#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32
from random import randint

def talker():
    pub = rospy.Publisher("chatter", Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = randint(1,2)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
