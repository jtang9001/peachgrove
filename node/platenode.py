#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import sys
from collections import deque, defaultdict
import traceback
from string import maketrans

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
        self.plateNumConf = {i: 0 for i in range(16)}
    
    def addPlate(self, plateNum, plateStr, numConf = 75, strConf = 75):
        for i in range(4):
            self.plates[plateNum - 1][i][plateStr[i]] += strConf
            self.plateNumConf[plateNum - 1] += numConf

    def renderPlates(self):
        plateStrs = {}
        usedPlateStrs = set()
        highestConfSpots = sorted(
            self.plateNumConf.iterkeys(), 
            key=lambda k: self.plateNumConf[k],
            reverse=True
        )

        for i in highestConfSpots:
            charFreqList = self.plates[i]
            plateStr = ''
            for charFreqDict in charFreqList:
                try:
                    mostFreqChar = max(charFreqDict.iterkeys(), key = lambda k: charFreqDict[k])
                    plateStr += mostFreqChar
                except ValueError:
                    pass
            if len(plateStr) != 0 and plateStr not in usedPlateStrs:
                usedPlateStrs.add(plateStr)
                plateStrs[i+1] = plateStr
            if len(usedPlateStrs) >= 6:
                break
        return plateStrs

def getAlnumChars(s):
    return ''.join(char for char in s if char.isalnum())


aFromCands = "IilZzJSsGTBOQoe"
numToCands = "111223556780000"

numFromCands = "12356780"
alphaToCands = "IZJSGZBO"

NUM_TO_ALPHA = maketrans(numFromCands, alphaToCands)
ALPHA_TO_NUM = maketrans(aFromCands, numToCands)


class PlateReader:

    def __init__(self):
        self.startTime = rospy.get_rostime()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback, buff_size=2**26, queue_size=3)
        self.image_pub = rospy.Publisher("/annotated_image_plates", Image, queue_size=1)
        self.framenum = 0
        self.lastGoodFrame = 0
        self.stackedPlates = np.zeros((100, 100, 3),np.uint8)
        self.plateStorage = PlateStorage()

    def callback(self,data):
        try:
            if rospy.get_rostime() - self.startTime > rospy.Duration.from_sec(4*60):
                rospy.logwarn_once(self.plateStorage.renderPlates())
                raise NoLicensePlatesException

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            rectPairs, threshedFrame = getPlates(cv_image)

            stackedPlates = None
            for rectPair in rectPairs:
                spotNum = None
                plateStr = None
                for rect in rectPair:
                    rect.perspectiveTransform()
                    rectStr, conf = rect.ocrFrame()
                    rectAlnum = getAlnumChars(rectStr)
                    #rospy.loginfo(rectAlnum + " - conf: " + str(conf))

                    if len(rectAlnum) == 3:
                        spotNum = int(rectAlnum[1:].translate(ALPHA_TO_NUM))
                        spotConf = conf

                    elif len(rectAlnum) == 4:
                        plateLetters = rectAlnum[:2].translate(NUM_TO_ALPHA)
                        plateNumbers = rectAlnum[2:].translate(ALPHA_TO_NUM)
                        plateStr = (plateLetters + plateNumbers).upper()
                        plateConf = conf

                    if stackedPlates is None:
                        stackedPlates = rect.threshedPersFrame_rgb
                    else:
                        stackedPlates = np.vstack([stackedPlates, rect.threshedPersFrame_rgb])

                if plateStr is not None and spotNum is not None:
                    rospy.loginfo(
                        "%s - %s, spot conf = %.2f, plate conf = %.2f" % (spotNum, plateStr, spotConf, plateConf)
                    )
                    self.plateStorage.addPlate(spotNum, plateStr, spotConf, plateConf)
                
            if stackedPlates is not None:
                self.stackedPlates = stackedPlates
                rospy.loginfo(self.plateStorage.renderPlates())
        
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