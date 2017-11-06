#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32MultiArray, String
from constants import robot_ID, field_ID

ser = None

def init_mainboard():
    global ser
    ser = serial.Serial("/dev/ttyACM3",9600,timeout = 0.002)
    ser.write('sd0:0:0:0\n')

def turn_on_motors(data):
    (f1, f2, f3, f4) = tuple(data[0:4])
    ser.write('sd%i:%i:%i:%i\n' % (round(f4*128), round(f2*128), round(f1*128), round(f3*128)))

def read_ref_commands():
    s = ser.read(100)
    #~ s = "ref:aBCSTOP----\n"
    find_ref = s.find("ref:")
    if find_ref != -1:
        s = s[find_ref+4:find_ref+16]
    else:
        s = ""
    c = 0
    if len(s) > 0 and s[0] == 'a' and len(s) >= 12:
        if s[1] == field_ID and (s[2] == robot_ID or s[2] == 'X'):
            # Command start
            if s[3:8] == "START":
                c = "START"
            elif s[3:7] == "STOP":
                c = "STOP"
            elif s[3:7] == "PING":
                c = "PING"
            ser.write('rf:a%c%cACK------\n' % (field_ID, robot_ID))
    return c

def __main__():
    rospy.init_node('hardware_interactor', anonymous=True)
    rospy.Subscriber("ToMotors", Int32MultiArray, turn_on_motors)
    pub = rospy.Publisher("FromRef", String, queue_size=10)
    init_mainboard()
    rate = rospy.Rate(1000) # 10hz
    while not rospy.is_shutdown():
        c = read_ref_commands()
        pub.publish(c)
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
