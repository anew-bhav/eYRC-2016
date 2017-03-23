import cv2
import numpy as np

def obstacle_detect(hsv,grid_map,m,n,grid_line_x,grid_line_y):

    lower = np.array([70, 123, 44], np.uint8)
    upper = np.array([93, 255, 255], np.uint8)

    mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CROSS, kernel)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True) [:6] #maximum 4 obstacles 
    no=0
    for i in contours:
        area = cv2.contourArea(i)
        peri = cv2.arcLength(i,True)
        if area>= 80:
            no=no+1
            approx = cv2.approxPolyDP(i,0.02*peri,True)
            cv2.fillPoly(mask, [approx], (255, 255, 255)) #filling contour with white color

    kernel = np.ones((21,21), np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # closing is done
    dilation = cv2.dilate(closing, kernel, iterations=1) # dilation
    
    for x in range(0, grid_line_x-1):
        X = x*m+(m/2)
        for y in range(0, grid_line_y-1):
            Y = y*n+(n/2)
            if mask[X, Y] >= 250 or x == 0 or x == grid_line_x-1:
                grid_map[x][y] = 0
                cv2.circle(hsv, (X,Y), 1, (0, 50, 200), -1)

    return grid_map,no
