##DISCLAIMER:
'''
* This code has been downloaded from
'''

####################################################################################################################################
'''
* Team ID:          CC#1822
* Author List:      Anubhav Jain, Pankhuri, Rahul Bareja
* Filename:         traversal
* Theme:            Cross A Crater
* Functions:        dis, grid_to_pixel, get_coordinate, getslope, pid_value, orientmove, bot_traverse, bot_route, traverse
* Global Variables: stepper, last_route_length, setpoint, last_proportional, integral, angle, Kp, Ki, Kd, route_length, route_path,
                    pos_x, pos_y, end_x, end_y, grid_line_x, grid_line_y, grid_map, destination, Bot_position, h, l, k, line_widthm,
                    line_widthn, ser
'''
####################################################################################################################################

import numpy as np
import cv2
import heapq   # priority queue
import serial  # library function for accessing xbee module
import math
import time

from recipe import *

stepper = 2

last_route_length = 0

setpoint = 0
last_proportional = 0
integral = 0

angle = 0

Kp = 0.8             # Proportional constant
Ki = 0.037425       # Integral constant
Kd = 0.72           # Derivative constant

###############################################
'''
* Function Name:	dis
* Input:		Two points co-ordinate
* Output:		distance between two points
* Logic:		math's rule of finding distance between two points
* Example Call:	dis(1,2,78,56)
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
* Example Call:	grid_to_pixel(3,3,30,33)
'''
##############################################
def grid_to_pixel(grid_x, grid_y, height, width):

    pixel_x = grid_x*height+height/2 	# stores the integer value of pixel's x coodinate
    pixel_y = grid_y*width+width/2 	# stores the integer value of pixel's y coodinate
    return pixel_x, pixel_y

##############################################
'''
* Function Name:	get_coordinate
* Input:		pixel_x: integer which stores pixel's x coordinates
*                       pixel_y: integer which stores pixel's y coordinates
*                       heigtht: integer, stores height of grid cell
*                       width: integer, stores width of grid cell
* Output:		returns grid cell's coordinates
* Logic:		Converts pixel coordinates into their specific grid coordinates under which those pixels lies.
* Example Call:	get_coordinate(233, 245, 30, 33)
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
* Example Call: orient_move(0.5,0.3, 4,3, 1,2, 12,14,3) 
'''
##############################################
def orientmove(slope_bot2cell, slope_botmarkers, bot_grid_x, bot_grid_y, route_x, route_y, distance_centre2cell, distance_other2cell, bot_distance):

    if distance_other2cell < 10 :                                       # check slave bot reaches destination if yes stop
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
                print 'data sent forward'
                
            elif pid_correction < -5:    # take right if error is negative
                if pid_correction > -250:
                    pid_correction = 255 + pid_correction
                    pid_correction = pid_correction/2+pid_correction%2
                    ser.write(chr(pid_correction)) # correction to motor
                    ser.write("\xff")              # command to turn right
                    print 'data sent turn right'
                    #time.sleep(0.1)
                        
                else:                   # fast right if error exceed min range
                    ser.write(chr(0))
                    ser.write("\xfb")  # right
                    print 'data sent fast right'
                    #time.sleep(0.35)

            else:
                if pid_correction < 250: # take left if error is positive
                    pid_correction = 255-pid_correction
                    pid_correction = pid_correction/2+pid_correction%2
                    ser.write(chr(pid_correction))# correction to speed
                    ser.write("\xfe")             # command to turn left
                    print 'data sent left'
                    #time.sleep(0.1)

                else:                    # fast left if error exceed max range
                    ser.write(chr(0))
                    ser.write("\xfc")
                    print 'data sent fast left'
                    #time.sleep(0.35)
        else:
            return 0

##############################################
'''
* Function Name:	bot_traverse
* Input:		route_path,destination co-ordinate,frame 
* Output:		path traversing sequence generated and a flag is sent to indicate if destination is reached
* Logic:		traverse path co-odinates sequentially as a local destination point to reach global destination point
'''
##############################################
def bot_traverse(route_path, dest_x, dest_y, frame):

    global stepper
    #print 'stepper', stepper, 'rl', len(route_path)
    #print 'route_bt', route_path

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
        return 0

    else:
        ser.write("\xfd")
        print "Reached Final Destination"
        return 1

###########################################################
'''
* Function Name:	bot_route
* Input:		frame
* Output:		store route path and change if new path is shorter returned from solve function
* Logic:		grid_start and grid_end send to solve the path and store it in route path
'''
############################################################
def bot_route(frame):

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

    #print 'pos',pos_x,pos_y,end_x,end_y
    if Bot_position[0][0] < (65/160)*w :
        dirs = 4
    else:
        dirs = 8

    route_length, route_path = solveGrid(grid_map,grid_line_x,grid_line_y,dirs,pos_x,pos_y,end_x,end_y)

    print 'route',route_path,'r', route_length

    if last_route_length == route_length:
        route_path = last_route_path

    last_route_length = route_length
    last_route_path = route_path

###################################################
'''
* Function Name:	traverse
* Input:                bot coordinates, destination coordinates, frame, lines of grid, grid map, and serial port object
* Logic:		capture video from cam and convert it to hsv for further video processing
'''
####################################################
def traverse(Bot_pos, dest_pos, frame, grid_x, grid_y, g_map, serl) :

    global Bot_position, destination_position
    global h,k,l
    global line_widthm, line_widthn
    global grid_line_x, grid_line_y
    global grid_map
    global ser

    grid_map = g_map
    Bot_position = Bot_pos
    destination_position = dest_pos
    grid_line_x = grid_x
    grid_line_y = grid_y
    ser = serl
    
    h, k, l = frame.shape
    line_widthm = h/(grid_line_x-1)       # length of a grid cell
    line_widthn = k/(grid_line_y-1)       # width of a grid cell
            
    cv2.line(frame, (destination_position[0][0], destination_position[0][1]), (Bot_position[0][0], Bot_position[0][1]), (255, 255, 0), 1)      # draw line between bot rear to destination
    cv2.line(frame, (Bot_position[0][0], Bot_position[0][1]), (Bot_position[1][0], Bot_position[1][1]), (0, 255, 255), 1)   # draw line between bot markers
       
    bot_route(frame)                    # call bot_route

    check = bot_traverse(route_path, end_x, end_y, frame)     # call bot_traverse to traverse path

    return check

############################################
## Close and exit
cv2.waitKey(0)
cv2.destroyAllWindows()
#################################################
