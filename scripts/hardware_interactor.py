#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32MultiArray, String
from constants import robot_ID, field_ID

ser = None
current_command = "START"

def init_mainboard():
    global ser
    ser = serial.Serial("/dev/ttyACM0",9600,timeout = 0.002)
    ser.write('sd:0:0:0\n')

def turn_on_motors(data):
    global current_command
    if current_command == "STOP":
        ser.write('sd:0:0:0\n')
        return
    (f1, f2, f3) = tuple(data[0:3])
    ser.write('sd:%i:%i:%i\n' % (round(f1*128), round(f3*128), round(f2*128)))
    #~ s = ser.read(100)
    #~ find_gs = s.find("gs:")
    #~ if find_gs != -1:
        #~ s = s[find_gs+3:find_gs+15]

def read_ref_commands():
    s = ser.read(100)
    #~ s = "ref:aBCSTOP----\n"
    find_ref = s.find("ref:")
    if find_ref != -1:
        s = s[find_ref+4:find_ref+16]
    else:
        s = ""
    c = ""
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
    global current_command
    rospy.init_node('hardware_interactor', anonymous=True)
    rospy.Subscriber("ToMotors", Int32MultiArray, turn_on_motors)
    pub = rospy.Publisher("FromRef", String, queue_size=10)
    pub2 = rospy.Publisher("FromMotors", Int32MultiArray, queue_size=10)
    init_mainboard()
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        c = read_ref_commands()
        if c:
            if c == "STOP" or c == "START":
                current_command = c
            pub.publish(c)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        if ser:
            ser.close()
