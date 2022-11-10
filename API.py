import server_comm as sc
import cv2
import time

udpip="192.168.145.41"
udpport=3760
obj = sc.Server(udpip, udpport) # UDP_IP and UDP_Port will be given on the day on testing and finals
'''
    ir = obj.recv_message(65535) # Gets the IR Sensor Readings <type:str>

    obj.send_command("front,dont") # Bot moves front without using the dropper
    obj.send_command("back,dont") # Bot moves back without using the dropper
    obj.send_command("fastright,dont") # Bot moves fast right front without using the droper
    obj.send_command("fastleft,dont") # Bot moves fast left without using the dropper
    obj.send_command("slowright,dont") # Bot moves slow right without using the dropper
    obj.send_command("slowleft,dont") # Bot moves slow left without using the dropper
    obj.send_command("stop,dont") # Bot stops without using the dropper
    obj.send_command("stop,drop") # Bot stops without and uses the dropper
'''
# cap = cv2.VideoCapture(2)
obj.send_command("front,dont")
time.sleep(1)
obj.send_command("stop,dont")


# while True:
#     rt, frame = cap.read()
#     if cv2.waitKey(1)==13:
#         break
#     cv2.imshow("aruco",frame)
#     cv2.waitKey(5)




