#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class TRAFFICDetector:
    def __init__(self):
        # print("traffic_detector!!!!!")
        # rospy.init_node("traffic_detector", anonymous=True)
        self.image_sub=rospy.Subscriber("/usb_cam/image_raw/compressed/compressed", CompressedImage, self.callback)
        self.traffic_msg=String()
        self.signal_pub=rospy.Publisher("/traffic_light", String, queue_size=10)
        self.count=0
        self.img_hsv=None

    def callback(self, msg):
        # print("callback")
        try:
            np_arr=np.fromstring(msg.data, np.uint8)
            img_bgr=cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
        self.img_hsv=cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
   

    def detect_signal(self):
        h=self.img_hsv.shape[0]
        
        lower_sig_b=np.array([[84, 0, 0]])
        upper_sig_b=np.array([123, 255,255])
        lower_sig_r=np.array([153, 0, 0])
        upper_sig_r=np.array([180, 255, 255])

        img_r=cv2.inRange(self.img_hsv, lower_sig_r, upper_sig_r)
        img_b=cv2.inRange(self.img_hsv, lower_sig_b, upper_sig_b)

        # None zero pixel counts
        pix_r=cv2.countNonZero(img_r)
        pix_b=cv2.countNonZero(img_b)

        # max_pixel count
        pix_max=np.max([pix_r, pix_b])
        idx_s=np.argmax([pix_r, pix_b])

        # print("RED:",pix_r)
        # print("BLUE:", pix_b)

        if pix_max>1800:
            while(self.count<10):
                self.count=self.count+1
                if pix_r>20000 and pix_r<180000:
                    self.traffic_msg.data="RED"
                    # print("RED")
                else:
                    self.traffic_msg.data="BLUE"
                    # print("BLUE")

                if self.count==10:
                    break
                    print("break")
        else:
            self.traffic_msg.data="None"
        
        
    def pub_signal(self):
        rospy.init_node("traffic_detector", anonymous=True)
        rate=rospy.Rate(20)
        self.detect_signal()

        self.signal_pub.publish(self.traffic_msg)
        print(self.traffic_msg)
        return self.traffic_msg.data
        rate.sleep()

    def run(self):
        # rospy.init_node("traffic_detector", anonymous=True)
        # traffic_detector=TRAFFICDetector()
        rate=rospy.Rate(20)

        # while not rospy.is_shutdown():
        if self.img_hsv is not None:
            self.detect_signal()
            self.pub_signal()
            rate.sleep()


# if __name__=='__main__':
#     try:
#         test_track =TRAFFICDetector()
#         test_track.start()
#     except rospy.ROSInterruptException:
#         pass
