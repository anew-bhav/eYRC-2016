########################################################################################################################
'''
* Team ID:          CC#1822
* Author List:      Anubhav Jain, Pankhuri, Rahul Bareja, Rhythm Vohra
* Filename:         main
* Theme:            Cross A Crater
* Functions:        reach_destination
* Global Variables: ser, grid_line_x, grid_line_y, grid_map, destination,Bot_position, h, l, k, line_widthm, line_widthn
'''
########################################################################################################################

import numpy as np
import cv2
import serial

from test_1_cam import *
from Bot_posn import *
from Cavity_detecn import *
from Obstacle_detecn import *
from bridge_decision import *
from traversal import *

ser = serial.Serial('COM6')         #opening port for communication with X-Bee

grid_line_x = 35                    #no. of grid line on x axis
grid_line_y = 35                    #no. of grid lines on y axis

grid_map = [[1 for i in range(grid_line_y)] for j in range(grid_line_x)]    #generating grid map

destination = [[0 for x in range(2)] for x in range(1)]         #generating map for location of destination

Bot_position = [[0 for x in range(2)] for x in range(2)]        #generating bot position map


#################################################################################
'''
* Function Name:    reach_destination
* Input:            video capturing object
* Output:           Bot reaches the destination
* Logic:            traverse the bot to destination by continuosly checking the distance between bot and destination
* Example Call: reach_destination(vid)
'''
#################################################################################
def reach_destination(cap):
    while (1):
        #time.sleep(1/80)                                # time between frames sampling is 1/80 sec.
        ret, frame = cap.read()                          # read frames from video
        frame,flag = test(frame)                         #cropping the region of interest i.e the region surrounded by wide black border of arena
        frame1 = cv2.GaussianBlur(frame,(5,5),1)         #smoothening the image
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)    #converting image frame to HSV
        Bot_position = Bot_pos(hsv)                     #DETECTING Bot Position using bot_position function
        if (traverse(Bot_position, destination, frame, grid_line_x, grid_line_y, grid_map, ser) == 1):      #traversal and breaking out of infinite loop if reached the destination
            break
#################################################################################
'''
* Function Name:    main function
* Logic:            Call all the modules and make the bot traverse on the arena and reach the base station
'''
#################################################################################
if __name__ == "__main__":

    global h, l, k
    global line_widthm, line_widthn
    cap = cv2.VideoCapture(0)
    
    ###########################################################################################################################
    '''this loop is used to prevent motion of bot and null evaluation and computation until the camera is not properly open'''
    ###########################################################################################################################
    
    while(1):
        ret,frame = cap.read()
        mean,std = cv2.meanStdDev(frame)
        if(std[1] > 60):
            break
        
    ret, frame = cap.read()    # read frames from video

    frame,flag = test(frame)
    frame1 = cv2.GaussianBlur(frame,(5,5),1)
    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)   # convert rgb coloured image to hsv image

    grid_map = [[1 for i in range(grid_line_y)] for j in range(grid_line_x)]    #grid_map to define path and avoid obstacles

    h, k, l = frame.shape
    line_widthm = h/(grid_line_x-1)       # length of a grid cell
    line_widthn = k/(grid_line_y-1)       # width of a grid cell

    #################################################################################
    '''Coordinates of the nodes on the crater region
          A1  ####################  A2
                        #
                        #
                        #
          A4  ##########I#########  A3
                        #
                        #
                        #
        T1  #########################  T2
            #                       #
            #                       #

                I = Intermediate
    '''
    #################################################################################
    
    T1 = [(47*k/160), (77*h/92)]
    T2 = [(47*k/160), (20*h/92)]
    Intermediate = [(29*k/160), (50*h/92)]
    A1 = [(11*k/160),(69*h/92)]
    A2 = [(11*k/160),(33*h/92)]
    A3 = [(29*k/160),(33*h/92)]
    A4 = [(29*k/160),(69*h/92)]
    base_station = [(148*k/160),(50*h/92)]
    terminal = [T1,T2]
    boulder = [A1,A2,A3,A4]
    
    cav_b1,cav_b2 = cavity_detect(hsv)      #function called to detect cavities
    grid_map,obs_no = obstacle_detect(hsv,grid_map,line_widthm,line_widthn,grid_line_x, grid_line_y)    #function to detect obstacles and hence modify grid_map
    #detection of digits
    
    boulder_map,bridge = run(len(cav_b1),len(cav_b2),obs_no-2,[0,3,0,8],11)     #function to choose between the 2 bridges
    
    if (boulder_map == -1):
        print 'Sum is not satisfied. Traversal not possible!'

    if bridge == 1:
        cavity = cav_b1
    else:
        cavity = cav_b2
        
    for i in range(len(cavity)):            #shift centroid of cavity so that bot doesn't fall
        cavity[i][0]-=25

    choice_terminal = terminal[bridge-1]    #coordinates of terminal of chosen bridge
    choice_boulder = []         #list of the boulders to be picked

    for i in range(len(boulder_map)):       #loop to add coordinates of boulders to the list
        if boulder_map[i]==1:
            choice_boulder.append(boulder[i])
    
    choice_boulder = choice_boulder[::-1]       #reverse the list, to pick farthest one first
    #destination[0][:] = Intermediate[:]
    #reach_destination(cap)

    #################################################################################
    '''loop to traverse along the arena till all the cavities are filled'''
    #################################################################################
          
    while choice_boulder != []:
        destination[0][:] = choice_boulder.pop()        #set the first boulder as destination
        print 'dest_main', destination[0][:]
        reach_destination(cap)          
        ser.write('\xA0')           #serial command to pick-up boulder
        destination[0][:] = Intermediate[:]     #come to intermediate point and then move to the terminalof selected bridge
        reach_destination(cap)    
        destination[0][:] = choice_terminal[:]
        reach_destination(cap)     
        destination[0] = cavity.pop()       #pop the cavity to be filled
        reach_destination(cap)      
        ser.write('\xA1')           #serial command to drop the boulder
        if cavity == []:            #check if cavities have been filled then move to the base station else move to the intermediate node
            destination[0][:] = [choice_terminal[0]+ (101*w/160), choice_terminal[1]]
            reach_destination(cap)    
            destination[0][:] = base_station[:]
            reach_destination(cap)
            ser.write('\xB0')       #start buzzer for 5 seconds
            time.sleep(5)
            ser.write('\xB1')
        else:
            ser.write('\xf2')       #serial command to move backwards
            time.sleep(1)
            ser.write('\xfc')
            time.sleep(1)
            destination[0][:] = choice_terminal[:]
            reach_destination(cap)
            destination[0][:] = Intermediate[:]
            reach_destination(cap)
                                       
    cv2.waitKey(0)           # exit from the loop or video
    
############################################
## Close and exit    
cv2.waitKey(0)
cv2.destroyAllWindows()
############################################
