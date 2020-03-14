#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('competition_2019t2')
import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import random
import time
from math import pi, atan2

class Region:
    def __init__(self, cornerA, cornerB, angleRange, target, angleOffset = 0):
        self.minX = min(cornerA[0], cornerB[0])
        self.minY = min(cornerA[1], cornerB[1])
        self.maxX = max(cornerA[0], cornerB[0])
        self.maxY = max(cornerA[1], cornerB[1])
        self.target = target
        self.angleRange = angleRange
        self.angleOffset = angleOffset

    def makePosition(self):
        return Position(self)

class Position:
    def __init__(self, region):
        self.x = random.uniform(region.minX, region.maxX)
        self.y = random.uniform(region.minY, region.maxY)

        #self.yaw = random.uniform(*region.angleRange)
        self.correctAngle = atan2((self.y - region.target[1]), (self.x - region.target[0])) + pi
        self.error = random.uniform(-1 * region.angleRange, region.angleRange) + region.angleOffset
        self.yaw = self.correctAngle + self.error

    def makeModelState(self):
        state_msg = ModelState()

        state_msg.model_name = "lab_robot"
        state_msg.pose.position.x = self.x
        state_msg.pose.position.y = self.y
        state_msg.pose.position.z = 0.165

        quat = quaternion_from_euler(0, 0, self.yaw)
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]

        # rospy.loginfo(self.x)
        # rospy.loginfo(self.y)
        rospy.loginfo(self.error)
        # rospy.loginfo(quat)

        return state_msg

REGIONS = [
    Region((-2.4, -2.4), (-1.75, 1.75), pi/4, (-2.1, 2.5)),
    Region((-2.4, 1.75), (-1.75, 2.3), pi/2, (2.5, 2.1), angleOffset=pi/4),
    Region((-1.75, 1.75), (1.75, 2.4), pi/4, (2.5, 2.1))
]

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.rate = rospy.Rate(0.5)
        self.counter = 0

    def callback(self,data):
        if self.counter > 1000:
            return

        regionnum = random.randint(0, len(REGIONS) - 1)
        region = REGIONS[regionnum]
        posn = region.makePosition()
        stateMsg = posn.makeModelState()

        rospy.wait_for_service("/gazebo/set_model_state")
        self.pub.publish(stateMsg)

        try:
            msg = rospy.wait_for_message("/rrbot/camera1/image_raw", Image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        
        outimg = "output/{counter}_{posn.error:.5f}_{regionnum}_{posn.x:.3f}_{posn.y:.3f}_{posn.yaw:.3f}.jpg".format(
            counter = self.counter, regionnum = regionnum, posn = posn) 
        saved = cv2.imwrite(outimg, cv_image)
        rospy.loginfo(outimg)
        # rospy.loginfo(saved)
        self.counter += 1
        self.rate.sleep()
        

def main(args):
    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)