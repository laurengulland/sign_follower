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
        # self.thresh_img = None
        # self.edged = None
        self.grid_cell = None
        self.crop_img = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window_1')
        cv2.namedWindow('video_window_2')
        cv2.namedWindow('video_window_3')

        self.center_x, self.center_y = 0,0
        self.radius = 0
        self.crop_left, self.crop_right, self.crop_top, self.crop_bottom = 0,0,0,0
        self.left_top, self.right_bottom = (0,0), (1,1)
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

        self.hsv_lb[0], self.hsv_lb[1], self.hsv_lb[2] = config.h_lower, config.s_lower, config.v_lower
        self.hsv_ub[0], self.hsv_ub[1], self.hsv_ub[2] = config.h_upper, config.s_upper, config.v_upper
        return config

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        self.bin_img = cv2.inRange(self.hsv_img, self.hsv_lb, self.hsv_ub)

        self.sign_bounding_box()
        # draw bounding box rectangle
        cv2.rectangle(self.cv_img, self.left_top, self.right_bottom, color=(0, 0, 255), thickness=5)
        cv2.rectangle(self.bin_img, self.left_top, self.right_bottom, color=(0, 0, 255), thickness=5)

        # left, top = left_top
        # right, bottom = right_bottom
        # crop bounding box to region of interest
        self.crop_left = int(0.8*(self.center_x-self.radius))
        self.crop_right = int(1.2*(self.center_x+self.radius))
        self.crop_top = int(0.8*(self.center_y-self.radius))
        self.crop_bottom = int(1.2*(self.center_y+self.radius))

        self.crop_img = self.cv_img[self.crop_top:self.crop_bottom, self.crop_left:self.crop_right]

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
        # grid_cell_w = int(self.bin_img.shape[0]/6)
        # grid_cell_h = int(self.bin_img.shape[1]/6)
        #
        # self.grid_cell = self.hsv_img[0:grid_cell_h, 0:grid_cell_w]

        ret,self.thresh_img = cv2.threshold(self.bin_img,0,255,0)
        im2,contours,hierarchy = cv2.findContours(self.bin_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        # print contours[0]
        x,y,w,h = cv2.boundingRect(cnt)
        # print x,y

        #Get pointcloud
        # bPoints, wPoints = [],[]
        # height,width = self.thresh_img.shape[:2]
        # idx = np.where(self.thresh_img==0)
        # for i,j in zip(idx[0],idx[1]):
        #     bPoints.append((i,j))
        #
        # idx = np.where(self.thresh_img==255)
        # for i,j in zip(idx[0],idx[1]):
        #     bPoints.append((i,j))
        #
        # bPoints,wPoints = np.array(bPoints),np.array(wPoints)
        # print wPoints.shape

        self.radius = 2
        moments = cv2.moments(self.thresh_img)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            # print int(self.center_x), int(self.center_y)
            self.radius = int(abs(self.center_y - y))
        cv2.circle(self.cv_img,(int(self.center_x), int(self.center_y)), self.radius, (0,0,255), 3)

        #
        # # #Trying a circle?
        # # (x2,y2),r = cv2.minEnclosingCircle(cnt)
        # # center = (int(x),int(y))
        # # radius = int(r)
        #Trying edges?
        #http://www.pyimagesearch.com/2014/04/21/building-pokedex-python-finding-game-boy-screen-step-4-6/
        # grey = cv2.bilateralFilter(self.hsv_img,11,17,17)
        # self.edged = cv2.Canny(grey,30,200)
        # print self.edged
        # (cnts,_) = cv2.findContours(self.edged.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # # cnts = sorted(cnts,key = cv2.contourArea, reverse=True)[:10]
        # screenCnt =
        cv2.rectangle(self.cv_img,(x,y),(x+w,y+h),(0,255,0),2)
        x = 0
        y = 0
        w = 0
        h = 0
        self.left_top = (x, y)
        self.right_bottom = (self.left_top[0]+w, self.left_top[1]+h)
        # print left_top, right_bottom
        # print w, h

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        r.sleep()
        while not rospy.is_shutdown():
            if not self.cv_img is None:
                # print "here"
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window_1', self.cv_img)
                if self.crop_img.any(): #won't break if crop_img is empty
                    cv2.imshow('video_window_2', self.crop_img)
                cv2.imshow('video_window_3', self.bin_img)

                cv2.waitKey(5)
            r.sleep()

# class TemplateMatcher(object,images):
#
#     def __init__(self):


if __name__ == '__main__':
    node = StreetSignRecognizer()
    node.run()
    # images = {
    #     "left": '../images/leftturn_box_small.png',
    #     "right": '../images/rightturn_box_small.png',
    #     "uturn": '../images/uturn_box_small.png'
    #     }
    # tm = TemplateMatcher(images)
    # scenes = [
    #     "../images/uturn_scene.jpg",
    #     "../images/leftturn_scene.jpg",
    #     "../images/rightturn_scene.jpg"
    #     ]
    #
    # for filename in scenes:
    #     scene_img = cv2.imread(filename, 0)
    #     pred = tm.predict(scene_img)
    #     print filename.split('/')[-1]
    #     print pred
