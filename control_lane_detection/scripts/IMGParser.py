#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

class IMGParser:
    def __init__(self):
        self.img=None
        self.set_cam(1)
    def set_cam(self, _index):
        self.cam=cv2.VideoCapture(int(_index))
    def get_image(self):
        ret, img=self.cam.read()
        return ret, img
    def get_bi_img(self):
        ret, img_bgr=self.get_image()
        img_hsv=cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([75, 0, 220])
        upper_wlane=np.array([175, 20, 225])
        img_wlane=cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        return img_wlane