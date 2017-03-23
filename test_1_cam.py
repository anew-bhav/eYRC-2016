import cv2
import numpy as np

def rectify(h):
    ''' this function put vertices of square we got, in clockwise order '''
    h = h.reshape((4,2))
    hnew = np.zeros((4,2),dtype = np.float32)

    add = h.sum(1)
    hnew[0] = h[np.argmin(add)]
    hnew[2] = h[np.argmax(add)]
		
    diff = np.diff(h,axis = 1)
    hnew[1] = h[np.argmin(diff)]
    hnew[3] = h[np.argmax(diff)]

    return hnew
	    
def test(img1):
    #convert image to grayscale and then apply blur and thresholding
    img2 = img1.copy()
    gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape[:2]
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
    #cv2.imshow('capture',img1)
    
    #find the perimeter and area of grayscale image
    img_peri=2*(height+width)
    img_area= height*width

    #find the contours and sort in decreasing order
    contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse= True) [:6]
            
    #find the desired contour by area thresholding
    cnt = None
    max_area = img_area #area of image
    for i in contours:
        area = cv2.contourArea(i)
        peri = cv2.arcLength(i,True)
        if area > img_area/2 and peri > img_peri/2:  #area n peri used as a filter
            #approximate the shape of contour
            approx = cv2.approxPolyDP(i,0.02*peri,True)
            #find the rectangle with least area
            if area < max_area and len(approx)==4:
                cnt = approx
                max_area = area
                
    #draw the contour detected
    cv2.drawContours(img1,[cnt],0,(0,255,0),3)
    
    #create mask and black out the undesired part
    mask = np.ones_like(img1)
    cv2.drawContours(mask,[cnt],0, 67, -1)
    
    #get the corners of the image
    xIm = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(xIm,4,0.01,10)
    corners=np.int0(corners)

    #the desired corners of the image
    list1=np.array([[0,0],[700,0],[700,400],[0,400]],np.float32)
    pts1=np.float32(corners)
    pts1=rectify(pts1) #arrange in clockwise order

    #transform the desired image and get new image
    M=cv2.getPerspectiveTransform(pts1,list1)
    img=cv2.warpPerspective(img2,M,(700,400))
    
    cv2.imwrite('cropped.jpg',img)
    return img, True
