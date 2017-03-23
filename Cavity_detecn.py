import cv2
import numpy as np

def get_key(lst):
    return lst[0]

def cavity_detect(hsv):

    h,w = hsv.shape [:2]
    
    cav_b1=[]
    cav_b2=[]
    
    lower = np.array([96, 85, 95])
    upper = np.array([116, 255, 255])
    
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CROSS, kernel)

    cv2.imwrite('mask.jpg',mask)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:7]
    
    for i in contours:
        area = cv2.contourArea(i)
        peri = cv2.arcLength(i,True)
        if area> 500:
            approx = cv2.approxPolyDP(i,0.02*peri,True)
            M = cv2.moments(approx)
            cx = int(M['m10']/M['m00']) 
            cy = int(M['m01']/M['m00'])
            #to check if cavity is in bridge 1 or bridge 2
            if cy > 0 and cy < (40*h/92):
                cav_b2.append(list([cx,cy])) 
            else:
                cav_b1.append(list([cx,cy]))
            
    cav_b1=sorted(cav_b1, key=get_key, reverse = True)
    cav_b2=sorted(cav_b2, key=get_key, reverse = True)
    
    return cav_b1,cav_b2
