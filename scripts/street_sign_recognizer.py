#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

from dynamic_reconfigure.server import Server
from sign_follower.cfg import SignFollowingConfig
import dynamic_reconfigure.client


class StreetSignRecognizer(object):
    """ This robot should recognize street signs """

    def __init__(self):
        """ Initialize the street sign reocgnizer """
        rospy.init_node('street_sign_recognizer')
        self.cv_img = None                        # the latest image from the camera
        self.bin_img = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window_1')
        cv2.namedWindow('video_window_2')

        rospy.Subscriber("/camera/image_raw", Image, self.process_image)

        # Dynamic Reconfigure
        srv = Server(SignFollowingConfig, self.configCallback)
        self.hsv_lb = np.array([14,231,139])
        self.hsv_ub = np.array([29,255,255])

    def configCallback(self,config,level): #dynamic reconfiguration callback
        print 'reconfiguring'
        rospy.loginfo("""Reconfigure Request: {h_lower}, {h_upper}, {s_lower}, {s_upper}, {v_lower}, {v_upper}""".format(**config))
        self.hsv_lb = np.array([14,231,139])
        self.hsv_ub = np.array([29,255,255])
        self.hsv_lb[0] = config.h_lower
        self.hsv_lb[1] = config.s_lower
        self.hsv_lb[2] = config.v_lower
        self.hsv_ub[0] = config.h_upper
        self.hsv_ub[1] = config.s_upper
        self.hsv_ub[2] = config.v_upper
        return config

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        self.bin_img = cv2.inRange(self.hsv_img, self.hsv_lb, self.hsv_ub)

        left_top, right_bottom = self.sign_bounding_box()
        # draw bounding box rectangle
        cv2.rectangle(self.cv_img, left_top, right_bottom, color=(0, 0, 255), thickness=5)

        left, top = left_top
        right, bottom = right_bottom
        # crop bounding box to region of interest
        cropped_sign = self.cv_img[top:bottom, left:right]

    def sign_bounding_box(self):
        """
        Returns
        -------
        (left_top, right_bottom) where left_top and right_bottom are tuples of (x_pixel, y_pixel)
            defining topleft and bottomright corners of the bounding box
        -------
        Questions to answer: what are distinguishing features? Where are similarities in color and
        geometry? What are different methods of generating candidate boxes? What defines a good box?
        """
        # TODO: YOUR SOLUTION HERE
        left_top = (200, 200)
        right_bottom = (400, 400)
        return left_top, right_bottom

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        r.sleep()
        while not rospy.is_shutdown():
            if not self.cv_img is None:
                print "here"
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window_1', self.cv_img)
                cv2.imshow('video_window_2', self.bin_img)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    node = StreetSignRecognizer()
    node.run()
