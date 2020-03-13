#! /usr/bin/env python
from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('competition_2019t2')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler
import random
from math import pi

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        quat = quaternion_from_euler(0,0,random.random() * 2 * pi)

        state_msg = ModelState()
        state_msg.model_name = "lab_robot"
        state_msg.pose.position.x = random.random()
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = random.random()
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]

        input("Press enter to continue")
        

def main(args):
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
    
main(sys.argv)