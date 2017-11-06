import serial
from constants import robot_ID, field_ID

ser = None

def init_mainboard():
    global ser
    ser = serial.Serial("/dev/ttyACM3",9600,timeout = 0.002)
    ser.write('sd0:0:0:0\n')

def turn_on_motors((f1, f2, f3)):
    ser.write('sd%i:%i:%i:%i\n' % (0, round(f2*128), round(f1*128), round(f3*128)))

def read_ref_commands():
    s = ser.read(100)
    if len(s) > 0:
        print "received ref command", s
    find_ref = s.find("ref:")
    if find_ref != -1:
        print "received ref command", s
        s = s[find_ref+4:find_ref+16]
        print "formatted s:", s
    else:
        s = ""
    c = 0
    if len(s) > 0 and s[0] == 'a' and len(s) >= 12:
        if s[1] == field_ID and (s[2] == robot_ID or s[2] == 'X'):
            # Command start
            if s[3:8] == "START":
                c = 1
            elif s[3:7] == "STOP":
                c = 2
            elif s[3:7] == "PING":
                c = 3
            ser.write('rf:a%c%cACK------\n' % (field_ID, robot_ID))
    return c

def __main__():
    init_mainboard()
    c = read_ref_commands()
