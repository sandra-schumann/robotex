import math
import numpy as np
import cv2
import copy
import serial
import time

cap = cv2.VideoCapture(0)

h = 32.0
a = 22.0
phi_deg = 51.0
phi = phi_deg*math.pi/180
dist_from_robot_midpoint = 9.0
ra_deg = 65.0
ra = ra_deg*math.pi/180
dist_correction = 9
ang_correction_deg = -1.5
turn_omega = 0.2
v = 0.2
ser = None

robot_ID = 'C'
field_ID = 'B'

def hinda_kaugust(h, a, phi, b_frac):
    alpha = math.atan(float(a)/h)
    f = math.sin(phi)*(h**2+a**2)**0.5/math.sin(math.pi/2-phi/2)
    b = f*b_frac
    d = (h**2 + a**2 + b**2 - 2*(h**2 + a**2)**0.5*b*math.cos(math.pi/2-phi/2))**0.5
    beta = math.asin(b*math.sin(math.pi/2-phi/2)/d)
    x = h*math.tan(alpha+beta)
    return x

def hinda_nurka(x, ra):
    h = 320/math.sin(ra/2)*math.cos(ra/2)
    return math.atan(x/h)

def get_motor_speeds(ax, ay, omega):
    f1 = ax*0.58 - ay*0.33 + omega*0.33
    f2 = -ax*0.58 - ay*0.33 + omega*0.33
    f3 = ay*0.67 + omega*0.33
    return f1, f2, f3

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

def process_image():
    try:
        # Take each frame
        _, frame = cap.read()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # define range of orange color in HSV
        lower_ball = np.array([5,200,200])
        upper_ball = np.array([15,255,255])
        #lower_ball = np.array([40, 70, 235])
        #upper_ball = np.array([60, 130, 255])
        
        # define range of blue color in HSV
        lower_basket = np.array([90,235,175])
        upper_basket = np.array([130,255,195])
        
        # Threshold the HSV image to get only appropriate colors
        ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
        # Bitwise-AND mask and original image
        ball_res = cv2.bitwise_and(frame,frame, mask= ball_mask)
        # Find ball midpoint
        M = cv2.moments(cv2.cvtColor(ball_res, cv2.COLOR_BGR2GRAY))
        ball_midpoint = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
        cv2.circle(res, ball_midpoint, 5, (255, 255, 255), -1)
        
        cv2.imshow('Camera image',cv2.bitwise_or(frame, ball_res))
    except:
        ball_midpoint = (0,0)
    
    return ball_midpoint

init_mainboard()

while(1):
    c = read_ref_commands()
    
    # Find the ball
    #~ ball_x, ball_y = process_image()
    ball_x, ball_y = (0, 0)
    ball_exists = True
    if ball_x == 0 and ball_y == 0:
        ball_exists = False
    
    if ball_exists:
        ball_distance = hinda_kaugust(h,a,phi,ball_y/480.) - dist_correction
        #~ print "ball is approximately at distance", ball_distance
        ball_angle = hinda_nurka(ball_x-320,ra)*180/math.pi - ang_correction_deg
        #~ print "ball is approximately at angle", ball_angle, "from the robot"
    else:
        ball_distance = 0
        ball_angle = 0
    
    # Turn towards ball
    omega = 0
    if c != 2 and (ball_angle > 1 or not ball_exists):
        #~ print "turn counterclockwise"
        omega = -turn_omega
    elif c != 2 and ball_angle < -1:
        #~ print "turn clockwise"
        omega = turn_omega
    else:
        pass
        #~ print "do not turn"
    
    # Move towards ball
    ax = 0
    if c != 2 and ball_exists and ball_distance > 24:
        #~ print "move forward"
        ax = v
    else:
        pass
        #~ print "do not move forward"
    
    #~ print "move with speeds", get_motor_speeds(ax, 0, omega)
    turn_on_motors(get_motor_speeds(ax, 0, omega))
    
    #~ time.sleep(0.05)
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
cv2.destroyAllWindows()
if ser:
    ser.close()
