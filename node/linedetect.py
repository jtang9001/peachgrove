from __future__ import division
import cv2

video = cv2.VideoCapture("raw_video_feed.mp4")

class NoContoursException(Exception):
    pass

def hsvThresh(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    satFrame = hsvFrame[:, :, 1]
    satBGRFrame = cv2.cvtColor(satFrame, cv2.COLOR_GRAY2BGR)
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
    return cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]


def getCenter(contour, yOffset = 0):
    moments = cv2.moments(contour)
    cX = int(moments["m10"] / moments["m00"])
    cY = int(moments["m01"] / moments["m00"])
    return (cX, cY + yOffset)

def drawCircle(frame, center, radius = 5):
    cv2.circle(frame, center, radius, (0,0,255), thickness=-1)

def analyze(frame):
    hsvFrame = hsvThresh(frame)
    binFrame = grayscaleThresh(getBottomPixels(hsvFrame, 100), 50)
    blurFrame = cv2.blur(binFrame, (5,5))

    contours = getContours(blurFrame)
    if len(contours) > 0:
        largestContour = max(contours, key = lambda contour: cv2.contourArea(contour))
        
    else:
        raise NoContoursException("No contours found") 

    center = getCenter(largestContour, yOffset=frame.shape[0] - 100)
    xCenter = center[0]
    xFrac = (xCenter - frame.shape[1] / 2) / frame.shape[1]
    drawCircle(frame, center, 20)

    return xFrac, frame
    
if __name__ == "__main__":
    cv2.namedWindow("Original frame")
    cv2.namedWindow("Cropped HSV frame")

    while video.isOpened():
        hasFrame, frame = video.read()

        if not hasFrame:
            print("Out of frames??")
            break

        hsvFrame = hsvThresh(frame)
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
        cv2.imshow("Cropped HSV frame", dispFrame)

        key = cv2.waitKey(50) & 0xFF
        
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break