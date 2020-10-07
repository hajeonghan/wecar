#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

class purePursuit:
    def __init__(self, lfd):
        self.is_look_forward_point=False
        self.vehicle_length=0.5
        
        self.lfd=lfd
        self.min_lfd=0.7
        self.max_lfd=1.2

    if self.is_look_forward_point:
        steering_deg=math.atan2((2*self.vehicle_length*math.sin(theta)), self.lfd)*180/math.pi

        self.steering=np.clip(steering_deg, -17, 17)/34+0.5
        print(self.steering)
        return self.steering

    else: