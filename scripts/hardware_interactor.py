#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32MultiArray, String, Int32
from constants import robot_ID, field_ID
import time

ser = None
current_command = "START"
sdata = ""
stime = None
prevstime = None
total_dists = [0,0,0]

def init_mainboard():
    global ser
    ser = serial.Serial("/dev/ttyACM0",9600,timeout = 0.002)
    ser.write('sd:0:0:0\n')

def turn_on_motors(data):
    global current_command, sdata, stime, ser
    if current_command == "STOP":
        ser.write('sd:0:0:0\n')
        print "stopping motors"
    else:
        (f1, f2, f3) = tuple(data.data[0:3])
        ser.write('sd:%i:%i:%i\n' % (round(f1), round(f3), round(f2)))
        print "turning motors at", f1, f2, f3
    #~ while True:
        #~ news = ser.read(100)
        #~ if news == "":
            #~ break
        #~ sdata += news
    #~ sdata += ser.read(100)
    #~ find_gs = sdata.find("gs:")
    #~ if find_gs != -1:
        #~ s = sdata[find_gs+3:find_gs+15]
        #~ newstime = time.time()
        #~ sdata = sdata[find_gs+15:]
        #~ print "got gs", s
        #~ if stime != None:
            #~ gs = s.split(':')
            #~ for i in range(len(gs)):
                #~ try:
                    #~ gs[i] = int(gs[i])
                #~ except:
                    #~ gs = gs[:i]
            #~ print "gs is", gs
            #~ print "time that passed was", newstime - stime
            #~ print "distances gone through:", (newstime-stime)*

def turn_on_thrower(speed):
    global current_command
    if current_command == "STOP":
        ser.write('d:50\n')
        print "stopping thrower"
        return
    ser.write('d:%i\n' % (speed.data))
    print "turning on thrower at", speed.data

def read_ref_commands():
    global sdata, ser, stime, prevstime
    while True:
        news = ser.read(100)
        prevstime = stime
        stime = time.time()
        if news == "":
            break
        sdata += news
    find_ref = sdata.find("ref:")
    if find_ref != -1:
        print "sdata is", sdata
        s = sdata[find_ref+4:find_ref+16]
        sdata = sdata[find_ref+16:]
        print "got ref", s
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
    global current_command, sdata, total_dists
    rospy.init_node('hardware_interactor', anonymous=True)
    rospy.Subscriber("ToMotors", Int32MultiArray, turn_on_motors)
    rospy.Subscriber("ToThrower", Int32, turn_on_thrower)
    pub = rospy.Publisher("FromRef", String, queue_size=10)
    pub2 = rospy.Publisher("FromMotors", Int32MultiArray, queue_size=10)
    init_mainboard()
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
        
        find_gs = sdata.find("gs:")
        if find_gs != -1:
            s = sdata[find_gs+3:find_gs+15]
            sdata = sdata[find_gs+15:]
            print "got gs", s
            if prevstime != None:
                gs = s.split(':')
                for i in range(len(gs)):
                    try:
                        gs[i] = int(gs[i])
                    except:
                        gs = gs[:i]
                print "gs is", gs
                print "time that passed was", stime - prevstime
                for i in range(len(gs)):
                    total_dists[i] += (stime - prevstime)*gs[i]*100
                print "total distance:", total_dists
        
        rate.sleep()

if __name__ == '__main__':
    try:
        __main__()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        if ser:
            ser.close()
