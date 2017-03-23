###########################################
### Import OpenCV
import numpy as np
import cv2
import heapq   # priority queue
import serial  # library function for accessing xbee module
import math
import time

from test_1_cam import *
from Bot_posn import *
from croping import *
from Cavity_detecn import *
from Obstacle_detecn import *
from recipe import *

ser = serial.Serial('COM6')

grid_line_x = 30
grid_line_y = 30
path_sample = 0
stepper = 2

grid_map = [[1 for i in range(grid_line_y)] for j in range(grid_line_x)]
destination_position = [[0 for x in range(2)] for x in range(1)]
destination_position[0][0]=126
destination_position[0][1]=300

Bot_position = [[0 for x in range(2)] for x in range(2)]

last_route_length = 0

setpoint = 0
last_proportional = 0
integral = 0

angle = 0

Kp = 0.8             # Proportional constant
Ki = 0.037425       # Integral constant
Kd = 0.72 

###############################################
'''
* Function Name : 	wall_detection
* Input : 		hsv image
* Output:		wall contours which are more dilated 
* Logic:		find contours and use morphological operation to dilate it                   
* Example Call:	wall_detection(hsv)
'''
############################################
def wall_detection(hsv):

    lower = np.array([70, 186, 41], np.uint8)
    upper = np.array([98, 255, 146], np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    #cv2.imshow('mask',mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:7]  # contours of walls is 7 max

    cv2.fillPoly(mask, contours, (255, 255, 255))  # filling contour with white color

    kernel = np.ones((60, 60), np.uint8)  # kernel for morphological operation
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # closing is done
    dilation = cv2.dilate(closing, kernel, iterations=1) # dilation

    return dilation

###############################################
'''
* Function Name:	grid_draw
* Input:		frame, number of horizontal and vertical lines
* Output:		wall contours which are more dilated 
* Logic:		find contours and use morphological operation to dilate it 
* Example Call:	grid_draw(frame,21,21)
'''
##############################################
def grid_draw(frame, m, n):    
    for x in range(0, m):    # drawing lines
        X = x*line_widthm
        cv2.line(frame, (0, X), (k, X), (0, 0, 0), 2)  
    for y in range(0, n):    # drawing lines
        Y = y*line_widthn
        cv2.line(frame, (Y, 0), (Y, h), (0, 0, 0), 2)
    return frame

###############################################
'''
* Function Name:	grid_map_of_walls
* Input:		dialted image of masked walls, grid_lines x,y
* Output:		Represent walls position on a grid by denoting 1 on respective places in grid_map
* Logic:		check if grid centre has value 1 than store 1 in grid_map else 0 and boundaries are also default walls
* Example Call:	grid_map_of_walls(dilation,21,21)
'''
#############################################################
def grid_map_of_walls(walls, grid_line_x, grid_line_y):

    global grid_map
    global last_grid_map

    for x in range(0, grid_line_x-1):
        X = x*line_widthm+(line_widthm/2)
        for y in range(0, grid_line_y-1):
            Y = y*line_widthn+(line_widthn/2)
            if walls[X, Y] >= 250 :
                grid_map[x][y] = 0
                cv2.circle(frame, (Y, X), 1, (0, 50, 200), -1)
        continue
    last_grid_map = grid_map

###############################################
'''
* Function Name:	route_path_draw
* Input:		frame
* Output:		drawing shortest path on frame
* Logic:		create line between each successive two points of path
* Example Call:	route_path_draw(frame)
'''
####################################
def route_path_draw(frame):
    for i in range(1, len(route_path)):
        (xa,ya)=route_path[i-1]
        (xb,yb)=route_path[i]
        cv2.line(frame, (xa*line_widthn+line_widthn/2, ya*line_widthm+line_widthm/2), (xb*line_widthn+line_widthn/2, yb*line_widthm+line_widthm/2), (255, 0, 0), 1)

###############################################
'''
* Function Name:	dis
* Input:		frTwo points co-ordinate
* Output:		distance between two points
* Logic:		math's rule of finding distance between two points
* Example Call:	route_path_draw(frame)
'''
############################################
def dis(x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))    # square root function is called
    dist = int(dist)                                     # dist:stores the integer value of the distance
    return dist
    
############################################
'''
* Function Name:	grid_to_pixel
* Input:		grid_x: integer which stores grid cell x coordinates
*                       grid_y: integer which stores grid cell y coordinates
*                       height: integer, stores height of grid cell
*                       width: integer, stores width of grid cell
* Output:		returns grid cell's pixel coordinates
* Logic:		converts grid cell's centre coordinates into pixel coodinates
* Example Call:	gridtopixel(3,3,30,33)
'''
##############################################
def grid_to_pixel(grid_x, grid_y, height, width):
    pixel_x = grid_x*height+height/2 	# stores the integer value of pixel's x coodinate
    pixel_y = grid_y*width+width/2 	# stores the integer value of pixel's y coodinate
    return pixel_x, pixel_y

##############################################
'''
* Function Name:	getcoor
* Input:		pixel_x: integer which stores pixel's x coordinates
*                       pixel_y: integer which stores pixel's y coordinates
*                       heigtht: integer, stores height of grid cell
*                       width: integer, stores width of grid cell
* Output:		returns grid cell's coordinates
* Logic:		Converts pixel coordinates into their specific grid coordinates under which those pixels lies.
* Example Call:	getcoor(233, 245, 30, 33)
'''   
################################################
def get_coordinate(x, y):

    X = 0
    Y = 0
    for i in range(0, grid_line_x):   # drawing lines
        X = X+line_widthn
        Y = 0
        for j in range(0, grid_line_y):   # drawing lines
            Y = Y+line_widthm
            if x <= X and y <= Y:
                return i, j
                break

#################################################
'''
* Function Name:	getslope
* Input:		Two points co-ordinate
* Output:		slope of line passing through points
* Logic:		coordinate geometry rule finding slope
* Example Call:	getslope(3, 4, 15, 16)
'''                           
#################################################
def getslope(x1, y1, x2, y2):
    m = 0
    if x2-x1 != 0:  # checking if slope is not infinte
        m = -(float)(y2-y1)/(x2-x1) # using slope function of coordinate geometry
        return m
    else:
        return 50   # m>50 for angle>88 degrees or (180-88), in case slope is approaching infinite

#################################################
'''
* Function Name:	pid_value
* Input:		error value as angle between line of bot markers an bot to destination
* Output:		correction in the value which added to motor speed
* Logic:		proportional, integral and derivative are give commulative correction
* Example Call:	pid_value(-30)
*
''' 
##################################################
def pid_value(error):
    global integral, last_proportional  # globalize the variables
        
    proportional = error - setpoint     # proportional control value
        
    integral = integral + proportional  # integral control value
    if(integral < -255):                # clamped integral control value
        integral = -255
    if(integral > 255):
        integral = 255
            
    derivative = proportional - last_proportional   # derivative control value

    last_proportional = proportional      # store last proportional value
        
    correction = Kp*proportional + Ki*integral + Kd*derivative  # correction value

    return correction

##############################################
'''
* Function Name:	orientmove
* Input:		slope bot to destination, slope of botmarkers,distance_centre2cell,distance_other2cell,bot_distance
* Output:		xbee commands to control the firebird V
* Logic:		pid_correction value is added to left and right motor speeds according to turn decisions
'''
##############################################
def orientmove(slope_bot2cell, slope_botmarkers, bot_grid_x, bot_grid_y, route_x, route_y, distance_centre2cell, distance_other2cell, bot_distance):

    if distance_other2cell < 20 :                                       # check slave bot reaches destination if yes stop
        ser.write("\xfd")
        print "Reached Near To Master Bot"
        return 1
    
    else:
        if slope_bot2cell*slope_botmarkers != -1:
            theta = math.atan((slope_bot2cell-slope_botmarkers)/(1+slope_bot2cell*slope_botmarkers))
            angle = int(math.degrees(theta))
            print 'angle:', angle

            print 'distances: ', distance_other2cell, distance_centre2cell, bot_distance

            if pow(distance_other2cell, 2) > pow(distance_centre2cell, 2) + pow(bot_distance, 2): # check orientation from grid cell
                if angle < 0:
                    angle = 180 + angle
                else:
                    angle = -180 + angle
            pid_correction = int(pid_value(angle))
            print 'Kp',Kp, 'Ki', Ki, 'Kd', Kd 

            if pid_correction < -255:
                pid_correction = -255
            if pid_correction > 255:
                pid_correction = 255

            if pid_correction > -5 and pid_correction < 5:  # go forward
                ser.write("\xf8")
                print 'data sent'
                
            elif pid_correction < -5:    # take right if error is negative
                if pid_correction > -250:
                    pid_correction = 255 + pid_correction
                    pid_correction = pid_correction/2+pid_correction%2
                    ser.write(chr(pid_correction)) # correction to motor
                    ser.write("\xff")              # command to turn right
                    print 'data sent p1'
                    #time.sleep(0.1)
                        
                else:                   # fast right if error exceed min range
                    ser.write(chr(0))
                    ser.write("\xfb")  # right
                    print 'data sent p2'
                    #time.sleep(0.35)

            else:
                if pid_correction < 250: # take left if error is positive
                    pid_correction = 255-pid_correction
                    pid_correction = pid_correction/2+pid_correction%2
                    ser.write(chr(pid_correction))# correction to speed
                    ser.write("\xfe")             # command to turn left
                    print 'data sent p3'
                    #time.sleep(0.1)

                else:                    # fast left if error exceed max range
                    ser.write(chr(0))
                    ser.write("\xfc")
                    print 'data sent p4'
                    #time.sleep(0.35)
        else:
            return 0

##############################################
'''
* Function Name:	bot_traverse
* Input:		route_path,destination co-ordinate,frame 
* Output:		path traversing sequence generated
* Logic:		traverse path co-odinates sequentially as a local destination point to reach global destination point
'''
##############################################
def bot_traverse(route_path, dest_x, dest_y, frame):
    global stepper
    print 'stepper', stepper, 'rl', len(route_path)
    print 'route_bt', route_path

    if stepper < route_length-1:
        (xa,ya)=route_path[stepper]
        print 'xa,ya', xa, ya
        X,Y= grid_to_pixel(xa, ya, line_widthn, line_widthm)  # X,Y are pixels of next grid coor
        print 'X,Y', X, Y
        rear_bot_x, rear_bot_y = get_coordinate(Bot_position[0][0], Bot_position[0][1])
        front_bot_x, front_bot_y = get_coordinate(Bot_position[1][0], Bot_position[1][1])

        d1 = dis(Bot_position[0][0], Bot_position[0][1], X, Y)
        d2 = dis(Bot_position[1][0], Bot_position[1][1], X, Y)
        d3 = dis(Bot_position[0][0], Bot_position[0][1], Bot_position[1][0], Bot_position[1][1])

        mid_x = (Bot_position[0][0]+Bot_position[1][0])/2         # mid point of bot center and other point
        mid_y = (Bot_position[0][1]+Bot_position[1][1])/2         # mid point of bot center and other point

        bot_grid_x, bot_grid_y = get_coordinate(mid_x, mid_y)

        cv2.circle(frame, (X,Y), 4, (255, 100, 100), -1)

        m1 = getslope(Bot_position[0][0], Bot_position[0][1], X,Y)
        m2 = getslope(Bot_position[0][0], Bot_position[0][1], Bot_position[1][0], Bot_position[1][1])

        if orientmove(m1, m2, rear_bot_x+1, rear_bot_y+1, xa+1, ya+1, d1, d2, d3) == 1: # bot reaches next coor
            stepper = stepper+1

    else:
        ser.write("\xfd")
        print "Reached Final Destination"

###########################################################
'''
* Function Name:	bot_route
* Input:		frame
* Output:		store route path and change if new path is shorter returned from solve function
* Logic:		grid_start and grid_end send to solve the path and store it in route path
'''
############################################################
def bot_route(frame):

    global path_sample
    global route_length, route_path
    global pos_x, pos_y, end_x, end_y
    global last_route_length
    global last_route_path

    #last_route_path=[]

    #grid_start = GridPoint((Bot_position[0][1]/line_widthm), (Bot_position[0][0]/line_widthn))   # reversing coordinates for compatible with coordinate system of matrix
    #grid_end = GridPoint((destination_position[0][1]/line_widthm), (destination_position[0][0]/line_widthn))

    h,w,c=frame.shape

    pos_x=(Bot_position[0][0]/line_widthn)
    pos_y=(Bot_position[0][1]/line_widthm)
    
    end_x=(destination_position[0][0]/line_widthn)
    end_y=(destination_position[0][1]/line_widthm)

    print 'pos',pos_x,pos_y
    if Bot_position[0][0] < (65/160)*w :
        dirs = 4
    else:
        dirs = 8
    #dirs=8

    route_length, route_path = solveGrid(grid_map,grid_line_x,grid_line_y,dirs,pos_x,pos_y,end_x,end_y)

    print 'route',route_path,'r', route_length

    if last_route_length == route_length:
        route_path = last_route_path

    last_route_length = route_length
    last_route_path = route_path

###################################################
'''
* Function Name:	main function
* Logic:		capture video from cam and convert it to hsv for further video processing
'''
####################################################
if __name__ == "__main__":
    global h, k, l
    global line_widthm, line_widthn

    cap = cv2.VideoCapture(0)
    
    while(1):
        ret,frame = cap.read()
        mean,std = cv2.meanStdDev(frame)
        if(std[1] > 60):
            break

    while 1:
        #time.sleep(1/80)     # time between frames sampling is 1/80 sec.
        ret, frame = cap.read()    # read frames from video

        frame,flag = test(frame)
        frame1 = cv2.GaussianBlur(frame,(5,5),1)
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)   # convert rgb coloured image to rgb image

        grid_map = [[1 for i in range(grid_line_y)] for j in range(grid_line_x)]
        
        Bot_position=(hsv)
        print 'bot_qq',Bot_position

        h, k, l = frame.shape
        line_widthm = h/(grid_line_x-1)       # length of a grid cell
        line_widthn = k/(grid_line_y-1)       # width of a grid cell
    
        #crop_image(frame)
        #cavity = cavity_detect(hsv,h)
        #grid_map,obstacle_no = obstacle_detect(hsv,grid_map,line_widthm,line_widthn,grid_line_x, grid_line_y) 

        frame=grid_draw(frame,grid_line_x,grid_line_y)
        
        cv2.line(frame, (destination_position[0][0], destination_position[0][1]), (Bot_position[0][0], Bot_position[0][1]), (255, 255, 0), 1)      # draw line between bot rear to destination
        cv2.line(frame, (Bot_position[0][0], Bot_position[0][1]), (Bot_position[1][0], Bot_position[1][1]), (0, 255, 255), 1)   # draw line between bot markers

        destination_slope = getslope(destination_position[0][0], destination_position[0][1], Bot_position[0][0], Bot_position[0][1])           # find the slope of line passing through bot rear and destination points
        bot_slope = getslope(Bot_position[0][0], Bot_position[0][1], Bot_position[1][0], Bot_position[1][1])                                   # find the slope of line passing through bot rear and bot front markers

        distance_centre2destination = dis(destination_position[0][0], destination_position[0][1], Bot_position[0][0], Bot_position[0][1])      # find distance bot rear to destination
        distance_other2destination = dis(destination_position[0][0], destination_position[0][1], Bot_position[1][0], Bot_position[1][1])       # find distance bot front to destination
        bot_distance = dis(Bot_position[0][0], Bot_position[0][1], Bot_position[1][0], Bot_position[1][1])                                     # find distance between bot markers

        destination_x, destination_y = get_coordinate(destination_position[0][0], destination_position[0][1])
        rear_bot_x, rear_bot_y = get_coordinate(Bot_position[0][0], Bot_position[0][1])
        front_bot_x, front_bot_y = get_coordinate(Bot_position[1][0], Bot_position[1][1])

          
        bot_route(frame)                    # call bot_route

        route_path_draw(frame)              # call route_path_draw to draw path

        bot_traverse(route_path, end_x, end_y, frame)     # call bot_traverse to traverse path

        cv2.imshow('frame', frame)           # show output on frame
        if cv2.waitKey(100) == 27:           # exit from the loop or video
            break

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
