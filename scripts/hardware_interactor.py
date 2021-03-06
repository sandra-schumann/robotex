#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Int32
from constants import robot_ID, field_ID
import time

ser = None
current_command = "START"
sdata = ""
stime = None
prevstime = None
gdata = ""

def init_mainboard():
    global ser
    print "initiating mainboard"
    ser = serial.Serial("/dev/ttyACM0",9600,timeout = 0.01,write_timeout = 0.01)
    #ser.write('sd:0:0:0\n')

def turn_on_motors(data):
    global current_command, sdata, stime, ser
    if current_command == "STOP":
        while 1:
            try:
                ser.write('sd:0:0:0\n')
                break
            except serial.SerialTimeoutException:
                ser.close()
                init_mainboard()
        print "stopping motors"
    else:
        (f1, f2, f3) = tuple(data.data[0:3])
        while 1:
            try:
                ser.write('sd:%i:%i:%i\n' % (round(f1), round(f3), round(f2)))
                break
            except serial.SerialTimeoutException:
                ser.close()
                init_mainboard()
        print "turning motors at", f1, f2, f3

def turn_on_thrower(speed):
    global current_command, ser
    if current_command == "STOP":
        while 1:
            try:
                ser.write('d:50\n')
                break
            except serial.SerialTimeoutException:
                ser.close()
                init_mainboard()
        print "stopping thrower"
        return
    while 1:
        try:
            ser.write('d:%i\n' % (speed.data))
            break
        except serial.SerialTimeoutException:
            ser.close()
            init_mainboard()
    print "turning on thrower at", speed.data

def read_ref_commands():
    global sdata, ser, stime, prevstime, gdata
    while True:
        news = ser.read(100)
        prevstime = stime
        stime = time.time()
        if news == "":
            break
        sdata += news
    find_ref = sdata.find("ref:")
    find_gs = sdata.find("gs:")
    if find_ref != -1:
        print "sdata", sdata
        s = sdata[find_ref+4:find_ref+16]
    else:
        s = ""
    if find_gs != -1:
        print "sdata", sdata
        gdata = sdata[find_gs+3:find_gs+16]
    if find_ref != -1 or find_gs != -1:
        sdata = sdata[max(find_ref, find_gs)+4:]
        print "got", s
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
            if s[2] == robot_ID:
                while 1:
                    try:
                        ser.write('rf:a%c%cACK------\n' % (field_ID, robot_ID))
                        break
                    except serial.SerialTimeoutException:
                        ser.close()
                        init_mainboard()
    return c

def __main__():
    global current_command, sdata, gdata, stime, prevstime
    rospy.init_node('hardware_interactor', anonymous=True)
    rospy.Subscriber("ToMotors", Int32MultiArray, turn_on_motors)
    rospy.Subscriber("ToThrower", Int32, turn_on_thrower)
    pub = rospy.Publisher("FromRef", String, queue_size=10)
    pub2 = rospy.Publisher("FromMotors", Float32MultiArray, queue_size=10)
    init_mainboard()
    ser.write('sd:0:0:0\n')
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        c = read_ref_commands()
        if c:
            if c == "START":
                current_command = c
            if c == "STOP":
                current_command = c
                turn_on_motors(Int32MultiArray(data=[0,0,0]))
                turn_on_thrower(0)
            pub.publish(c)
        
        if gdata:
            gs = gdata.split(':')
            gdists = []
            for g in gs:
                try:
                    gdists.append(int(g)*(stime-prevstime))
                except:
                    break
            rospy.loginfo(Float32MultiArray(data=gdists))
            pub2.publish(Float32MultiArray(data=gdists))
            gdata = ""
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException, e:
        print e
        if ser:
            ser.close()
    except serial.SerialException, e:
        print e
        if ser:
            ser.close()
    except KeyboardInterrupt, e:
        print e
        if ser:
            ser.close()
