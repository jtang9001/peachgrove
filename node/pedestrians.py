import cv2
from math import pi
import numpy as np

def hasPedestrian(frame):
    
    #TODO: taking an bGR OpenCV frame as input, return true if there 
    # is a pedestrian in the frame (ie. should the car stop)
    
    # what do the pedestrians look like in the simulation: 
    # NOTE: For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]. 
    # Different softwares use different scales. 
    # If you are comparing OpenCV values with them, you need to normalize these ranges.
    
    #define some constants
    brownThresh = 0
    
    #Convert from BGR to HSV color-space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    #TODO: change to range of brown of pedestrian 
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image for a range of blue color (to get only blue colors)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise_AND on mask and original frame
    # In openCV, the value of the colour black is 0, so black + anycolour = anycolour
    # This will assign any pixels in our colour range a value 1 and everything else 0 (black)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    
    #count how many pixels in our range of brown
    #totalBrownpixels = cv.countNonZero(res)
    
    #find CM location of pedestrian (within frame)
    width = res.shape()[1]
    height = res.shape()[0]
    
    i_accumulator = 0
    i_count = 0
    for i in range(0,width) :
        for j in range(0,height) :
            if res[i,j] = 255 :
                i_accumulated += i
                i_count += 1
    
    pedestrian_CM = i_accumulated/i_count
    
    if pedestrian_CM in range(10,width-10) :
        return True
    return False
    
    #keep track of previous (20?) frames to track movement
    #use deque => if full (keep track of size), remove at one end before adding to top
    # monitor where white is in sea of black
    # crosses, stops, crosses bakc, stops

    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
   
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
