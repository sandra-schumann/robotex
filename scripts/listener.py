#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

def hinda_nurka(x):
    x -= 320
    ra = 80.0*math.pi/180
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "\nGoal at angle " + str(hinda_nurka(data.data[2])))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("GoalPos", Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
