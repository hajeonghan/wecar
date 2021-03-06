#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from utils import warp_image
from utils import BEVTransform, STOPLineEstimator
from std_msgs.msg import Float64

class IMGParser:
    def __init__(self):
        self.image_sub=rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_wlane=None
        self.source_prop=np.float32([[0.05, 0.65],
                                    [0.5-0.15, 0.52],
                                    [0.5+0.15, 0.52],
                                    [1-0.05, 0.65]
                                    ])
    def callback(self, msg):
        try:
            np_arr=np.fromstring(msg.data, np.uint8)
            img_bgr=cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv=cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([20,0,240])
        upper_wlane=np.array([40,15,255])
        self.img_wlane=cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        self.img_wlane[int(0.7*img_hsv.shape[0]): ,:]=0

        # img_wlane=cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
        # img_concat=np.concatenate([img_bgr, img_hsv, img_wlane], axis=1)
        # img_warp=warp_image(img_wlane, self.source_prop)

        # cv2.imshow("Image window", img_warp)
        # cv2.waitKey(1)

if __name__=='__main__':
    rp=rospkg.RosPack()
    currentPath=rp.get_path("stop_line_detection")
    with open(os.path.join(currentPath, "sensor/sensor_params.json"), 'r') as fp:
        sensor_params=json.load(fp)
    params_cam=sensor_params["params_cam"]
    rospy.init_node("image_parser", anonymous=True)
    image_parser=IMGParser()
    bev_op=BEVTransform(params_cam=params_cam)
   
    sline_detector=STOPLineEstimator()
    # curve_learner=CURVEFit(order=3)
    # ctrller=purePursuit(lfd=0.8)
    rate=rospy.Rate(30)

    while not rospy.is_shutdown():
        if image_parser.img_wlane is not None:
            lane_pts=bev_op.recon_lane_pts(image_parser.img_wlane)
            sline_detector.get_x_points(lane_pts)
            sline_detector.estimate_dist(0.3)
            # sline_detector.visualize_dist()
            sline_detector.pub_sline()
            rate.sleep()    








    # while not rospy.is_shutdown():
    #     if image_parser.img_wlane is not None:
    #     # if image_parser.edges in not None:
    #         img_warp=bev_op.warp_bev_img(image_parser.img_wlane)
    #         # img_warp=bev_op.warp_bev_img(image_parser.edges)
    #         lane_pts=bev_op.recon_lane_pts(image_parser.img_wlane)

    #         x_pred, y_pred_l, y_pred_r=curve_learner.fit_curve(lane_pts)

    #         ctrller.steering_angle(x_pred, y_pred_l, y_pred_r)
    #         ctrller.pub_cmd()

    #         xyl, xyr=bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

    #         img_warp1=draw_lane_img(img_warp, xyl[:, 0].astype(np.int32),
    #                                         xyl[:, 1].astype(np.int32),
    #                                         xyr[:, 0].astype(np.int32),
    #                                         xyr[:, 1].astype(np.int32))

    #         cv2.imshow("Image window", img_warp1)
    #         cv2.waitKey(1)

    #     rate.sleep()
    # rospy.spin()
