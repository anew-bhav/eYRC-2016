########################################################################################################################
'''
* Team ID:          CC#1822
* Author List:      Anubhav Jain
* Filename:         Bot_posn
* Theme:            Cross A Crater
* Functions:        Bot_pos
* Global Variables: None
'''
########################################################################################################################

import numpy as np
import cv2

###############################################
'''
* Function Name:	Bot_pos
* Input:		image in HSV format
* Output:		coordinates of the bot
* Logic:		find centroid of contour of bot with markers of red led panel
'''
############################################

def Bot_pos(hsv):

    lower = np.array([0,152,185])       # Lower HSV value for bot_markers
    upper = np.array([255,255,255])     # Higher HSV value range for bot_markers

    mask = cv2.inRange(hsv, lower, upper)   # masking of bot_markers of red colour
        
    kernel= np.zeros((3,3),np.uint8)
    kernel=kernel.fill(0)
    mask = cv2.dilate(mask, kernel, iterations=7)   # dilate the image to get good contours
        
    bot_position = [[0 for i in range(2)] for j in range(2)]
        
    try:            
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        for i in contours:
            peri = cv2.arcLength(i,True)
            approx = cv2.approxPolyDP(i,0.02*peri,True)
            M = cv2.moments(approx)      # centroid of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            bot_position[j][0]=cx       # store centroids
            bot_position[j][1]=cy

    except:
        pass

    return bot_position     # return the position of centroids
