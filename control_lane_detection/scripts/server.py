# !/usr/bin/env python
# echo_server.pys
# -*- coding:utf-8 -*-

from socket import *
from controller import simple_controller
from ss import IMGParser
s = socket(AF_INET, SOCK_STREAM)
s.setsockopt(SOL_SOCKET, SO_REUSEADDR,1)
s.bind(("0.0.0.0", 1301))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)

parkingG=None
exitG=None
while True:
    data = conn.recv(1024)
    print('Received Data :', data)
    msg_recv = ""

    for ch in data:
        # print(ord(ch))
        if ord(ch) != 0 and ord(ch) != 7 and ord(ch)!=4:
            msg_recv += str(ch)
    
    print (msg_recv)
    
    if msg_recv=="exit":
        print(" asd")
        exitG = simple_controller()
        # parkingG.control=False
    elif msg_recv=="parking":
        print(" asd")
        parkingG = IMGParser()
        # parkingG.control=False

    #print('Received from', addr)
    #print('Recived: ', data)
    send_msg_recv = "echo"
    conn.send(send_msg_recv)

conn.close()
s.close()



