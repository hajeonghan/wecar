#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

if params_cam["ENGINE"]=="UNITY":
    self.alpha_r=np.deg2rad(params_cam["FOV"]/2)
    self.fc_y=params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    self.alpha_c=np.arctan2(params_cam["WIDTH"]/2, self.fc_y)

    self.fc_x=self.fc_y

elif params_cam["ENGINE"]=="LOGITECH":
    self.fc_y=params_cam["HEIGHT"]/2*3.67
    self.alpha_c=np.arctan2(params["WIDTH"]/2, self.fc_y)
    self.fc_x=self.fc_y
    self.alpha_r=np.arctan2(params_cam["HEIGHT"]/2, self.fc_x)
else:
    self.alpha_c=np.deg2rad(params_cam["FOV"]/2)
    self.fc_x=params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    self.alpha_r=np.arctan2(params_cam["HEIGHT"]/2, self.fc_x)
    self.fc_y=self.fc_x
    
