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

from vanishpt import analyze, NoVanishingPointException
from plates import getPlates, PlateRect
import pedestrians


P_COEFF = -4
I_COEFF = -0.01
D_COEFF = 0
INTEGRAL_LENGTH = 40
MINSPEED = 0.02
MAXSPEED = 0.4
def getSpeedFromError(error):
    if error < 0:
        error *= -1
    return max(MINSPEED, MAXSPEED-error)

class image_converter:

    def __init__(self):
        self.startTime = rospy.get_rostime()

        rospy.sleep(3.) #give some time to read plate in starting position

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/annotated_image_vanishing_pt", Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("/annotated_image_mask", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw", Image, self.callback)
        self.integral = deque(maxlen=INTEGRAL_LENGTH)
        self.odometer = 0
        self.lengths = 0
        self.heading = 0
        self.lastTickTime = rospy.get_rostime()
        self.tickDuration = 0
        self.frame = None
        self.frame2 = None
        self.move = Twist()
        self.localTurnHeading = 0

    def callback(self,data):

        try:
            # Record heading and odometer with time * velocity
            currentTime = rospy.get_rostime()
            self.tickDuration = (currentTime - self.lastTickTime).to_sec()
            self.lastTickTime = currentTime
            self.odometer += self.move.linear.x * self.tickDuration
            self.heading += self.move.angular.z * self.tickDuration
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.move = Twist()
        try:
            #get error for PID driving
            xFrac, vanishPtFrame = analyze(cv_image)

            #detect license plates
            rectPairs, threshedFrame = getPlates(cv_image)
            for rectPair in rectPairs:
                for rect in rectPair:
                    rect.labelOnFrame(vanishPtFrame)

            cv2.putText(
                vanishPtFrame, "Pairs:" + str(len(rectPairs)), (20,90), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0), thickness=2
            )

            self.frame = vanishPtFrame

            self.integral.append(xFrac)
            intTerm = sum(self.integral)

            turn_start = 2.4 if self.lengths % 4 == 1 else 2.6
            turn_minimum_radians = -14.5 if self.lengths % 4 == 1 else -14

            #detect pedestrian
            hasPedestrian, maskFrame, pedX, pedY = pedestrians.hasPedestrian(cv_image)
            #rospy.loginfo(hasPedestrian)
            self.frame2 = cv2.cvtColor(maskFrame, cv2.COLOR_GRAY2BGR)

            # twirl override
            if self.lengths % 2 == 1 and self.localTurnHeading > turn_minimum_radians and turn_start < self.odometer < turn_start + 0.25:
                #rospy.loginfo("In turning override")
                self.twirl()

            # override for time expiry
            elif rospy.get_rostime() - self.startTime > rospy.Duration.from_sec(4*60):
                rospy.logwarn_once("Done!")
                self.move.linear.x = 0
                self.move.angular.z = 0

            # override for pedestrian
            elif hasPedestrian:
                cv2.circle(self.frame, (pedX, pedY), 7, (0,200,255), thickness=2)
                self.move.linear.x = 0
                self.move.angular.z = 0

            # drive normally
            else:
                self.localTurnHeading = 0
                self.move.linear.x = getSpeedFromError(xFrac)
                self.move.angular.z = xFrac*P_COEFF + intTerm*I_COEFF

        except NoVanishingPointException:
            if self.localTurnHeading != 0:
                self.twirl()
            else:
                self.turnCorner()
            self.frame = cv_image

        except Exception:
            rospy.logwarn(traceback.format_exc())
            self.turnCorner()
            self.frame = cv_image
            
        finally:
            # publish debugging images and send Twist message
            self.pub.publish(self.move) #comment out to drive robot manually
            odomStr = "%(length)d: OD %(odom).2f, HD %(head).2f" % {"odom": self.odometer, "head": self.localTurnHeading, "length": self.lengths}
            cv2.putText(
                self.frame, odomStr, (20,50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255,0,0), thickness=2
            )
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.frame2, "bgr8"))

    def turnCorner(self):
        # hardcoded right turn behavior
        self.integral.clear()
        self.move.linear.x = 0.05
        self.move.angular.z = -0.6
        if self.odometer > 3.5:
            self.odometer = 0
            self.lengths += 1
            rospy.loginfo("Now on length: ")
            rospy.loginfo(self.lengths)
            
    def twirl(self, ang_vel = -0.6):
        # hardcoded twirl behavior
        self.integral.clear()
        self.move.linear.x = 0.01
        self.move.angular.z = ang_vel
        self.localTurnHeading += self.move.angular.z * self.tickDuration
                
        

def main(args):
    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)