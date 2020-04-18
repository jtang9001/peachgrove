from __future__ import print_function
from __future__ import division

import cv2
import numpy as np
import itertools
pi = np.pi

GREEN = (0,255,0)
BLUE = (255,0,0)
RED = (0,0,255)
WHITE = (255,255,255)

class NoVanishingPointException(Exception):
    pass

class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        if x2 == x1:
            self.slope = np.inf
        else:
            self.slope = (y2-y1)/(x2-x1)

    def evalForX(self, x):
        return self.slope*(x - self.x1) + self.y1

    def solveForY(self, y):
        return (y - self.y1)/self.slope + self.x1
    
    def findIsectWith(self, other):
        if self.slope == other.slope:
            return None
        isectX = (other.y1 - self.y1 - other.slope*other.x1 + self.slope*self.x1)/(self.slope - other.slope)
        return (isectX, self.evalForX(isectX))

    def draw(self, frame, color = GREEN, thickness = 1):
        cv2.line(
            frame, (self.x1, self.y1), (self.x2, self.y2),
            color, thickness
        )
    
    def drawInf(self, frame, color = GREEN, thickness = 1):
        height, width = frame.shape[:2]
        infStartX = -100
        infStartY = int(round(self.evalForX(infStartX)))
        infEndX = int(width + 100)
        infEndY = int(round(self.evalForX(infEndX)))

        cv2.line(
            frame, (infStartX, infStartY), (infEndX, infEndY),
            color, thickness
        )

def auto_canny(image, sigma=0.5):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    # return the edged image
    #edged = cv2.Canny(image, 50, 100)
    return edged

def drawLinesOnBlank(lines, width, height):
    canvas = np.zeros((height, width, 3), np.uint8)
    for line in lines:
        line.drawInf(canvas, color = WHITE)
    return canvas

def findVanishingPt(lines, width, height):
    uniqueLines = [lines[0]]
    for line in lines:
        addLine = True

        for refLine in uniqueLines:
            if 0.9*refLine.slope < line.slope < 1.1*refLine.slope:
                addLine = False
                break

        if addLine:
            uniqueLines.append(line)

    isects = []
    weights = []
    for lineA, lineB in itertools.combinations(uniqueLines, 2):
        isect = lineA.findIsectWith(lineB)
        if isect is not None and 0 <= isect[0] <= width and height*0.35 <= isect[1] <= height*0.65:
            isects.append(isect)
            weights.append(abs(lineA.slope - lineB.slope))
    
    return np.average(np.array(isects), axis = 0, weights=weights), uniqueLines



def analyze(frame):
    height, width = frame.shape[:2]

    grayImg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edgedImg = auto_canny(grayImg)

    lineThreshold = 75
    minLineLength = 100
    maxLineGap = 10
    lines = cv2.HoughLinesP(edgedImg, 1, pi/180, lineThreshold, minLineLength, maxLineGap)

    if lines is None:
        raise NoVanishingPointException
    
    #retFrame = cv2.cvtColor(edgedImg, cv2.COLOR_GRAY2BGR)
    retFrame = frame.copy()

    diagLines = []
    for line in lines:
        #x1, y1, x2, y2 = line[0]
        lineObj = Line(*line[0])
        if 0.1 < abs(lineObj.slope) < 10:
            diagLines.append(lineObj)

    try:
        vanishPt, uniqueLines = findVanishingPt(diagLines, width, height)
    except (ZeroDivisionError, IndexError):
        raise NoVanishingPointException

    for line in uniqueLines:
        line.drawInf(retFrame)
    cv2.circle(retFrame, (int(round(vanishPt[0])), int(round(vanishPt[1]))), 5, RED, -1)

    #retFrame = drawLinesOnBlank(diagLines, width, height)

    return (vanishPt[0] / width) - 0.5, retFrame
