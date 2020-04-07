#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import sys
from collections import deque, defaultdict
import traceback

import numpy as np

import roslib
roslib.load_manifest('competition_2019t2')
import rospy
import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from plates import getPlates, PlateRect

class NoLicensePlatesException(Exception):
    pass

class PlateStorage:
    def __init__(self):
        self.plates = [[defaultdict(int) for i in range(4)] for j in range(16)]
    
    def addPlate(self, plateNum, plateStr, conf = 75):
        for i in range(4):
            self.plates[plateNum - 1][i][plateStr[i]] += conf

    def renderPlates(self):
        plateStrs = {}
        for i in range(len(self.plates)):
            charFreqList = self.plates[i]
            plateStr = ''
            for charFreqDict in charFreqList:
                try:
                    mostFreqChar = max(charFreqDict.iterkeys(), key = lambda k: charFreqDict[k])
                    plateStr += mostFreqChar
                except ValueError:
                    pass
            if len(plateStr) != 0:
                plateStrs[i+1] = plateStr
        return plateStrs

def getAlnumChars(s):
    return ''.join((char for char in s if char.isalnum()))

CHAR_CONF_CHART = {
    "1": ["I", "i", "l"],
    "2": ["Z", "z"],
    "3": ["J"],
    "5": ["S", "s"],
    "6": ["G"],
    "0": ["O", "o", "e"]
}

def getNumFromAlpha(alphaChar):
    for key, value in CHAR_CONF_CHART.iteritems():
        if alphaChar in value:
            return key
        else:
            return alphaChar
    
def getAlphaFromNum(numChar):
    if numChar in CHAR_CONF_CHART:
        return CHAR_CONF_CHART[numChar][0]
    else:
        return numChar

def convertStrToInt(s):
    return int("".join(getNumFromAlpha(x) for x in s))

def convertIntToStr(s):
    return "".join(getAlphaFromNum(x) for x in s)


class PlateReader:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback, buff_size=2**26, queue_size=3)
        self.image_pub = rospy.Publisher("/annotated_image_plates", Image, queue_size=1)
        self.framenum = 0
        self.lastGoodFrame = 0
        self.stackedPlates = np.zeros((100, 100, 3),np.uint8)
        self.plateStorage = PlateStorage()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            rectPairs, threshedFrame = getPlates(cv_image)

            #cv2.drawContours(threshedFrame, [rect.contour for rect in rects], -1, (0,255,0), 2)
            #frame = threshedFrame

            self.framenum += 1
            if len(rectPairs) != 0:
                rospy.loginfo("Frame " + str(self.framenum))
                # if self.framenum - self.lastGoodFrame == 1:
                #     #skip every other frame?
                #     raise NoLicensePlatesException

                # self.lastGoodFrame = self.framenum
                rospy.loginfo(self.plateStorage.renderPlates())
                #rospy.loginfo(self.plateStorage.plates)

            stackedPlates = None
            for rectPair in rectPairs:
                spotNum = None
                plateStr = None
                for rect in rectPair:
                    rect.perspectiveTransform()
                    rectStr, conf = rect.ocrFrame()
                    rectAlnum = getAlnumChars(rectStr)
                    rospy.loginfo(rectAlnum + " / conf: " + str(conf))

                    if len(rectAlnum) == 3:
                        spotNum = convertStrToInt(rectAlnum[1:])

                    elif len(rectAlnum) == 4:
                        plateLetters = convertIntToStr(rectAlnum[:2])
                        plateNumbers = convertStrToInt(rectAlnum[2:])
                        plateStr = plateLetters + str(plateNumbers)

                    if stackedPlates is None:
                        stackedPlates = rect.threshedPersFrame_rgb
                    else:
                        stackedPlates = np.vstack([stackedPlates, rect.threshedPersFrame_rgb])

                if plateStr is not None and spotNum is not None:
                    self.plateStorage.addPlate(spotNum, plateStr, conf)
                
            if stackedPlates is not None:
                self.stackedPlates = stackedPlates
        
        except NoLicensePlatesException:
            pass
        
        except Exception:
            rospy.logwarn(traceback.format_exc())
            
        finally:
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.stackedPlates, "bgr8"))
        

def main(args):
    
    rospy.init_node('plate_reader', anonymous=True)
    plate_reader = PlateReader()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)