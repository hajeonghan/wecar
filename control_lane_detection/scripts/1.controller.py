#!/usr/bin/env python
# echo_server.py
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        # rospy.init_node('simple_controller',anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud,queue_size=1)

        self.control=True
        self.laser_data=0

        self.stop=0

        print("control start!")

        while not rospy.is_shutdown():
            rospy.spin()


    def laser_callback(self,msg):
        pcd = PointCloud()
        motor_msg = Float64()
        servo_value = Float64()
        pcd.header.frame_id = msg.header.frame_id
        angle = 0
        servo_value = 0.555

        for r in msg.ranges:
                tmp_point= Point32()
                tmp_point.x = r*cos(angle)
                tmp_point.y = r*sin(angle)
                angle = angle + (1.0/180*pi)
                if r<2 :
                    pcd.points.append(tmp_point)
                    max_right=0
                    max_left=0
                    max_front=0
                    ready=0
                    
                    point_get = []  

        for pd in range(0, 360):
            # Range so big
            if str(msg.ranges[pd]) == 'inf':
                point_get.append(float(0.0))
            else:
                point_get.append(float(msg.ranges[pd]))

        # #right
        if point_get[89]>point_get[90] and point_get[89]>point_get[91]:
            max_right=point_get[89]
        elif point_get[90]>point_get[89] and point_get[90]>point_get[91]:
            max_right=point_get[90]
        elif point_get[91]>point_get[89] and point_get[91]>point_get[90]:
            max_right=point_get[91]

        # left
        if point_get[269]>point_get[270] and point_get[269]>point_get[271]:
            max_left=point_get[269]
        elif point_get[270]>point_get[269] and point_get[270]>point_get[271]:
            max_left=point_get[270]
        elif point_get[271]>point_get[269] and point_get[271]>point_get[270]:
            max_leftt=point_get[271]
        
        # front
        if point_get[179]>point_get[180] and point_get[179]>point_get[181]:
            max_front=point_get[179]
        elif point_get[180]>point_get[179] and point_get[180]>point_get[181]:
            max_front=point_get[180]
        elif point_get[181]>point_get[179] and point_get[181]>point_get[180]:
            max_front=point_get[180]

        print("max_right: ", max_right)
        print("max_left: ", max_left)
        print("max_front: ",max_front)
        if max_right == 0 and max_left == 0 and max_front==0:
            motor_msg.data = 0
            print("00000000000")

        # out
        if max_front > 1.3:
            servo_value = 0.555
            motor_msg.data=1000
        # stop
        elif 0 <max_front < 1.3:
            motor_msg.data = 0
            print("ok")
            self.control=False
            rospy.signal_shutdown("reason")
            
            
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_value)
        self.pcd_pub.publish(pcd)

         

# if __name__ == '__main__':
#     try:
#         test_track = simple_controller()
#     except rospy.ROSInterruptException:
#         pass
 

