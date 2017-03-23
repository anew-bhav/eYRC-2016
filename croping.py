from __future__ import division
import cv2
import numpy as np

def crop_image(img):
    h,w,c = img.shape

    wCrater = (65/160)*w
    craterRegion = img[0:h,0:wCrater]

    #pixels for bridge region
    wBridgeRegion = (70/160)*w

    cropDimBridge = [wCrater,wBridgeRegion+wCrater]
    bridgeRegion = img[0:h,cropDimBridge[0]:cropDimBridge[1]]

    # pixels for base station
    wBaseStation  = (25/160)*w
    baseStation = img[0:h,cropDimBridge[1]:cropDimBridge[1]+wBaseStation]
    cv2.imwrite('crater.jpg',craterRegion)
    cv2.imwrite('base.jpg',baseStation)
    cv2.imwrite('bridge.jpg',bridgeRegion)

    h,w,c = craterRegion.shape

    hdig= (15/72)*h
    digi1 = craterRegion[0:hdig,0:(36/65)*w]
    cv2.imwrite('digi1.jpg',digi1)

    digi2 = craterRegion[(h-hdig):h,0:(36/65)*w]
    cv2.imwrite('digi2.jpg',digi2)

    h,w,c = bridgeRegion.shape
    bridge2 = bridgeRegion[0:(40/92)*h, 0:w]
    cv2.imwrite('B2.jpg',bridge2)

    bridge1 = bridgeRegion[(62/92)*h:h, 0:w]
    cv2.imwrite('B1.jpg',bridge1)
    
    #65x92 crater region 25x92 base station 70x92 bridge region
    return craterRegion
    '''dimesions = []

    cv2.waitKey(0)
    cv2.destroyAllWindows()'''
