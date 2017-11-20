#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32MultiArray
from random import randint

def talker():
    pub = rospy.Publisher("GoTo", Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        robot_pos = Int32MultiArray(data=[ int(x) for x in raw_input("Go to point: ").split() if x ])
        #~ rospy.loginfo(robot_pos)
        pub.publish(robot_pos)
        #~ rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
