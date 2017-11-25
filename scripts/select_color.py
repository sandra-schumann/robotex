import numpy as np
import cv2
import copy

print "OpenCV version", cv2.__version__

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

_, frame = cap.read()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

selected_colors = []
color_range_low = (255,255,255)
color_range_high = (0,0,0)

cross_mask = np.zeros((500, 500, 3), np.uint8)
rect_mask = np.zeros((500, 500, 3), np.uint8)

cv2.namedWindow('image')
cv2.namedWindow('hsv_graph')
cv2.createTrackbar('S','hsv_graph',5,5,nothing)

def update_color_range():
    global selected_colors, color_range_low, color_range_high
    if selected_colors:
        color_range_low = np.array(
                          (min(color_range_low[0],reduce((lambda x, y: min(x,y)), [ color[0] for color in selected_colors ])),
                           min(color_range_low[1],reduce((lambda x, y: min(x,y)), [ color[1] for color in selected_colors ])),
                           min(color_range_low[2],reduce((lambda x, y: min(x,y)), [ color[2] for color in selected_colors ]))))
        color_range_high = np.array(
                          (max(color_range_low[0],reduce((lambda x, y: max(x,y)), [ color[0] for color in selected_colors ])),
                           max(color_range_low[1],reduce((lambda x, y: max(x,y)), [ color[1] for color in selected_colors ])),
                           max(color_range_low[2],reduce((lambda x, y: max(x,y)), [ color[2] for color in selected_colors ]))))
    else:
        color_range_low = np.array((180,255,255))
        color_range_high = np.array((0,0,0))

def undo_color_selection():
    global selected_colors
    if selected_colors != []:
        selected_colors.pop()
    update_color_range()

def draw_color_range(s):
    rect_mask = np.zeros((500, 500, 3), np.uint8)
    if selected_colors != []:
        cv2.rectangle(rect_mask, (color_range_low[0]*500/180, color_range_low[2]*500/255),
                                 (color_range_high[0]*500/180, color_range_high[2]*500/255),
                                 (255,255,255),2)
    return rect_mask

def select_color(event,x,y,flags,param):
    global cross_mask
    
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_color = tuple(map(np.uint8, hsv[y][x]))
        cv2.setTrackbarPos('S','hsv_graph',int(round(selected_color[1]*5/255.)))
        midpos = [selected_color[0]*500/180, selected_color[2]*500/255]
        cross_mask = np.zeros((500, 500, 3), np.uint8)
        cv2.line(cross_mask,(midpos[0]-5,midpos[1]-5),(midpos[0]+5,midpos[1]+5),(255,255,255),2)
        cv2.line(cross_mask,(midpos[0]-5,midpos[1]+5),(midpos[0]+5,midpos[1]-5),(255,255,255),2)
        selected_colors.append(selected_color)
        update_color_range()

cv2.setMouseCallback('image',select_color)

while(1):
    # Take each frame
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    h_gradient = np.ones((500,1), dtype=np.uint8)*np.linspace(0, 180, 500, dtype=np.uint8)
    v_gradient = np.rot90(np.ones((500,1), dtype=np.uint8)*np.linspace(255, 0, 500, dtype=np.uint8))
    s_pos = cv2.getTrackbarPos('S','hsv_graph')
    s = np.uint8(s_pos*255/5.)*np.ones((500,500), dtype=np.uint8)
    rect_mask = draw_color_range(s_pos)
    hsv_color = cv2.merge((h_gradient, s, v_gradient))
    rgb_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)
    ball_res = np.zeros(frame.shape, np.uint8)
    ones = cv2.bitwise_not(np.zeros(frame.shape, np.uint8))
    if selected_colors != []:
        try:
            color_mask = cv2.inRange(hsv, color_range_low, color_range_high)
        except:
            print "well this is weird"
            print color_range_low
            print color_range_high
        ball_res = cv2.bitwise_and(ones,ones,mask=color_mask)
    
    cv2.imshow('image', cv2.add(frame,ball_res))
    cv2.imshow('hsv_graph', cv2.bitwise_or(cv2.bitwise_and(rgb_color,cv2.bitwise_not(cross_mask)),rect_mask))
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    elif k == ord('u'):
        undo_color_selection()
    elif k == ord('s'):
        fout = open("selected_color_range.txt", 'w')
        low_str = str(color_range_low[0]) + ' ' + str(color_range_low[1]) + ' ' + str(color_range_low[2])
        high_str = str(color_range_high[0]) + ' ' + str(color_range_high[1]) + ' ' + str(color_range_high[2])
        fout.write(low_str + '\n' + high_str + '\n')
        fout.close()
    elif k == ord('b'):
        fout = open("ball_color.txt", 'w')
        low_str = str(color_range_low[0]) + ' ' + str(color_range_low[1]) + ' ' + str(color_range_low[2])
        high_str = str(color_range_high[0]) + ' ' + str(color_range_high[1]) + ' ' + str(color_range_high[2])
        fout.write(low_str + '\n' + high_str + '\n')
        fout.close()
    elif k == ord('g'):
        fout = open("goal_color.txt", 'w')
        low_str = str(color_range_low[0]) + ' ' + str(color_range_low[1]) + ' ' + str(color_range_low[2])
        high_str = str(color_range_high[0]) + ' ' + str(color_range_high[1]) + ' ' + str(color_range_high[2])
        fout.write(low_str + '\n' + high_str + '\n')
        fout.close()
    elif k == ord('f'):
        fout = open("field_color.txt", 'w')
        low_str = str(color_range_low[0]) + ' ' + str(color_range_low[1]) + ' ' + str(color_range_low[2])
        high_str = str(color_range_high[0]) + ' ' + str(color_range_high[1]) + ' ' + str(color_range_high[2])
        fout.write(low_str + '\n' + high_str + '\n')
        fout.close()
    elif k == ord('l'):
        fin = open("selected_color_range.txt", 'r')
        t = fin.read().splitlines()
        color_range_low = np.array([int(n) for n in t[0].split()])
        color_range_high = np.array([int(n) for n in t[1].split()])
        selected_colors.append(color_range_low)
        selected_colors.append(color_range_high)
        fin.close()

cv2.destroyAllWindows()
