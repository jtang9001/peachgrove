import cv2
from math import pi
import numpy as np

def hasPedestrian(frame):
    
    #TODO: taking an bGR OpenCV frame as input, return if there 
    # is a pedestrian in the frame (ie. should the car stop)
    
    # what do the pedestrians look like in the simulation: 
    # NOTE: For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]. 
    # Different softwares use different scales. 
    # If you are comparing OpenCV values with them, you need to normalize these ranges.
    
    #Convert from BGR to HSV color-space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image for a range of blue color (to get only blue colors)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    
    #next step is to count pixels of our desired shade

    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
        
    return false
    
#from lab 2
    binFrame = grayscaleThresh(getBottomPixels(hsvFrame, 100), 50)
    blurFrame = cv2.blur(binFrame, (5,5))

    contours = getContours(blurFrame)
    if len(contours) > 0:
        largestContour = max(contours, key = lambda contour: cv2.contourArea(contour))
    else:
        continue

    dispFrame = cv2.cvtColor(blurFrame, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(dispFrame, [largestContour], -1, (0,255,0), 3)
    drawCircle(frame, getCenter(largestContour, yOffset=frame.shape[0] - 100), 20)

    cv2.imshow("Original frame", frame)
    cv2.imshow("Contour frame", dispFrame)
    
#some functions from lab 2
def hsvThresh(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    satFrame = hsvFrame[:, :, 1]
    satBGRFrame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return satFrame

def getBottomPixels(frame, nPixels = 100):
    yCoords = frame.shape[0]
    return frame[yCoords - nPixels:yCoords, :]

# "binarizes" the frame. Only 0 or 255 are left possible.
def grayscaleThresh(frame, threshold = 50):
    _, threshedFrame = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)
    return threshedFrame

# findContours requires binary frame
def getContours(frame):
    return cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

def getCenter(contour, yOffset = 0):
    moments = cv2.moments(contour)
    cX = int(moments["m10"] / moments["m00"])
    cY = int(moments["m01"] / moments["m00"])
    return (cX, cY + yOffset)

def drawCircle(frame, center, radius = 5):
    cv2.circle(frame, center, radius, (0,0,255), thickness=-1)
