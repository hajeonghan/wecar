# !/usr/bin/env python
# echo_server.pys
# -*- coding:utf-8 -*-

from socket import *
from std_msgs.msg import String
from controller_left import left_controller
from controller_right import right_controller
from ss import IMGParser
from traffic_detector import TRAFFICDetector

print("waitttttt")
s = socket(AF_INET, SOCK_STREAM)
s.setsockopt(SOL_SOCKET, SO_REUSEADDR,1)
s.bind(("0.0.0.0", 1302))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)



parkingG=None
exitG=None
# inOut=None
arr=[]
# arr=[0]*10

while True:
    print("come in !!!!!")
    data = conn.recv(1024)
    print('Received Data :', data)
    msg_recv = ""
    

    for ch in data:
        # print(ord(ch))
        if ord(ch) != 0 and ord(ch) != 7 and ord(ch)!=4:
            msg_recv += str(ch)
    
    print (msg_recv)

    if msg_recv=="open":
        print("open ok")
        inOut=TRAFFICDetector()
        for i in range(10):
            send=String()
            send=inOut.pub_signal()
            arr.append(send)
            print(send)

        if  "RED" in send:
            send_msg_recv = "IN"
            conn.send(send_msg_recv)
            print("send IN!!!!!!!!!!")
        elif "BLUE" in send:
            send_msg_recv = "OUT"
            conn.send(send_msg_recv)
            print("send out!!!!!!!!!")
    else:
        if  "right_exit" in msg_recv:
            print("right")
            exitG = right_controller()
            print(exitG.control)
            if exitG.control==False:
                send_msg_recv="OK"
                conn.send(send_msg_recv)
                print("ridarrrrrrrrrrrrr")
        elif  "left_exit" in msg_recv:
            print("left")
            exitG = left_controller()
            print(exitG.control)
            if exitG.control==False:
                send_msg_recv="OK"
                conn.send(send_msg_recv)
                print("ridarrrrrrrrrrrrr")

        elif "parking" in msg_recv:
            print(" asd")
            parkingG = IMGParser()
            # parkingG.control=False
            
        send_msg_recv = "echo"
        conn.send(send_msg_recv)

conn.close()
s.close()



