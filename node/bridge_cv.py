#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import sys
from collections import deque
import traceback

import roslib
roslib.load_manifest('competition_2019t2')
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError

from vanishpt import analyze

P_COEFF = -2
I_COEFF = -0.03
D_COEFF = 0
INTEGRAL_LENGTH = 40
MINSPEED = 0.02
def getSpeedFromError(error):
    if error < 0:
        error *= -1
    return max(MINSPEED, 0.2-error)

class image_converter:

    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/annotated_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback)
        self.integral = deque(maxlen=INTEGRAL_LENGTH)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        move = Twist()
        try:
            xFrac, frame = analyze(cv_image)

            self.integral.append(xFrac)
            intTerm = sum(self.integral)
            try:
                derivTerm = self.integral[-1] - self.integral[-3]
            except Exception:
                derivTerm = 0

            move.linear.x = getSpeedFromError(xFrac)
            move.angular.z = xFrac*P_COEFF + intTerm*I_COEFF + derivTerm * D_COEFF
            # pidStr = "P = %(error).2f, I = %(integral).2f, D = %(deriv).2f" % {"error": xFrac, "integral": intTerm, "deriv": derivTerm}
            # outStr = "v = %(vel).2f, w = %(ang).2f" % {"ang": move.angular.z, "vel": move.linear.x}
            # cv2.putText(frame, pidStr, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), thickness=1)
            # cv2.putText(frame, outStr, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), thickness=1)
        except Exception:
            rospy.logwarn(traceback.format_exc())
            self.integral.clear()
            frame = cv_image
            move.linear.x = 0
            move.angular.z = -0.3
            cv2.putText(frame, "No vanishing point", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), thickness=3)

        try:
            self.pub.publish(move)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)