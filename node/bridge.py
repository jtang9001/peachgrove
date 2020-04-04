#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import sys
from collections import deque
import traceback
import time
from multiprocessing import Process, Pipe

import roslib
roslib.load_manifest('competition_2019t2')
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError

from vanishpt import analyze
from plates import getPlates, PlateRect
import pedestrians

P_COEFF = -2
I_COEFF = -0.02
D_COEFF = 0
INTEGRAL_LENGTH = 50
MINSPEED = 0.02
MAXSPEED = 0.2
def getSpeedFromError(error):
    if error < 0:
        error *= -1
    return max(MINSPEED, MAXSPEED-error)

class image_converter:

    def __init__(self, conn):
        self.startTime = rospy.get_rostime()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/annotated_image_vanishing_pt", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback)
        self.integral = deque(maxlen=INTEGRAL_LENGTH)
        self.odometer = 0
        self.lengths = 0
        self.connToOcr = conn

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        move = Twist()
        try:
            xFrac, vanishPtFrame = analyze(cv_image)
            # rects, threshedFrame = getPlates(cv_image)

            # cv2.drawContours(threshedFrame, [rect.contour for rect in rects], -1, (0,255,0), 2)
            frame = vanishPtFrame
            #self.connToOcr.send(cv_image)

            self.integral.append(xFrac)
            intTerm = sum(self.integral)

            # try:
            #     derivTerm = self.integral[-1] - self.integral[-3]
            # except Exception:
            #     derivTerm = 0

            if rospy.get_rostime() - self.startTime < rospy.Duration.from_sec(20) or pedestrians.hasPedestrian(cv_image):
                move.linear.x = 0
                move.angular.z = 0

            else:
                move.linear.x = getSpeedFromError(xFrac)
                move.angular.z = xFrac*P_COEFF + intTerm*I_COEFF# + derivTerm * D_COEFF

            self.odometer += move.linear.x
            # pidStr = "P = %(error).2f, I = %(integral).2f, D = %(deriv).2f" % {"error": xFrac, "integral": intTerm, "deriv": derivTerm}
            # outStr = "v = %(vel).2f, w = %(ang).2f" % {"ang": move.angular.z, "vel": move.linear.x}
            # cv2.putText(frame, pidStr, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), thickness=1)
            cv2.putText(frame, str(round(self.odometer, 2)), (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), thickness=2)
        
        except Exception:
            rospy.logwarn(traceback.format_exc())
            self.integral.clear()
            frame = cv_image
            move.linear.x = 0.01
            move.angular.z = -0.3
            cv2.putText(frame, "No vanishing point", (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), thickness=2)
            if self.odometer > 40:
                self.odometer = 0
                self.lengths += 1
                rospy.loginfo("Now on lap: ")
                rospy.loginfo(self.lengths)
            
        finally:
            self.pub.publish(move)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def ocrPlates(conn):
    frameNum = 0
    while True:
        if conn.poll():
            img = conn.recv()
            rects, threshedFrame = getPlates(img)
            frameNum += 1
            if len(rects) != 0:
                rospy.loginfo("Frame " + str(frameNum))
            for rect in rects:
                rect.perspectiveTransform()
                rospy.loginfo(rect.ocrFrame())
        else:
            continue
        

def main(args):
    conn1, conn2 = Pipe()
    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter(conn1)

    p = Process(target=ocrPlates, args=(conn2,))
    #p.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)