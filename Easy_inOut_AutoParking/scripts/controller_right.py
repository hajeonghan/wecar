#!/usr/bin/env python
# echo_server.py
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class right_controller:
    def __init__(self):
        # rospy.init_node('simple_controller',anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud,queue_size=1)

        self.control=True
        self.laser_data=0

        self.stop=0
        self.start = 0

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
                    right=0
                    left=0
                    front=0
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
            right=point_get[89]
        elif point_get[90]>point_get[89] and point_get[90]>point_get[91]:
            right=point_get[90]
        elif point_get[91]>point_get[89] and point_get[91]>point_get[90]:
            right=point_get[91]

        # left
        if point_get[269]>point_get[270] and point_get[269]>point_get[271]:
            left=point_get[269]
        elif point_get[270]>point_get[269] and point_get[270]>point_get[271]:
            left=point_get[270]
        elif point_get[271]>point_get[269] and point_get[271]>point_get[270]:
            leftt=point_get[271]
        
        # front
        if point_get[179]>point_get[180] and point_get[179]>point_get[181]:
            front=point_get[179]
        elif point_get[180]>point_get[179] and point_get[180]>point_get[181]:
            front=point_get[180]
        elif point_get[181]>point_get[179] and point_get[181]>point_get[180]:
            front=point_get[180]

        print("right: ", right)
        print("left: ", left)
        print("front: ",front)
        if right == 0 and left == 0 and front==0:
            motor_msg.data = 0
            print("00000000000")

        # go ahead
        if self.start == 0:
            if 0 < front < 1.0:
                self.start =1 

            else:
                servo_value=0.5555

                motor_msg.data=1000
        # right
        elif self.start == 1:
            if 1.2 < left < 1.3:
                servo_value = 0.555
                motor_msg.data = 0
                print("ok")
                self.control=False
                rospy.signal_shutdown("reason")
            else:
                print("right!!!!!!!!!!!")
                servo_value = 0.99
                motor_msg.data = 1000
            
            
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_value)
        self.pcd_pub.publish(pcd)

         

# if __name__ == '__main__':
#     try:
#         test_track = simple_controller()
#     except rospy.ROSInterruptException:
#         pass
 

