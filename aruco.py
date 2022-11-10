import cv2
import cv2.aruco as aruco
import math
import numpy as np
from scipy.spatial.transform import Rotation
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

cap = cv2.VideoCapture(2)

obj.send_command("front,dont")
time.sleep(1)
obj.send_command("slowright,dont")
time.sleep(0.5)
obj.send_command("front,dont")
time.sleep(1.7)

obj.send_command("slowleft,dont")
time.sleep(0.5)
obj.send_command("slowleft,dont")
time.sleep(0.5)
# obj.send_command("front,dont")
# time.sleep(1)
# obj.send_command("slowright,dont")
# time.sleep(1)
# obj.send_command("front,dont")
# time.sleep(1)
# obj.send_command("slowright,dont")
# time.sleep(1)
obj.send_command("stop,drop")





def eulerQuaternion(x, y, z, w):
    roll = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
    pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
    yaw = math.asin(2*x*y + 2*z*w)
    return roll, pitch, yaw

def findXYZandRPY(corners, cameraDist, cameraMatrix):
    if len(corners) > 0:
        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0][0], corners[0][0][1], corners[0][0][2], corners[0][0][3]
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))


        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, abs(topRight[0]-topLeft[0]), cameraMatrix=cameraMatrix, distCoeffs=cameraDist)

        transform_translation_x = tvecs[0][0][0]
        transform_translation_y = tvecs[0][0][1]
        transform_translation_z = tvecs[0][0][2]

        translation = [round(transform_translation_x, 4),round(transform_translation_y, 4), round(transform_translation_z, 4)]

        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0]))[0]
        r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()

        transform_rotation_x = quat[0]
        transform_rotation_y = quat[1]
        transform_rotation_z = quat[2]
        transform_rotation_w = quat[3]

        roll, pitch, yaw = eulerQuaternion(transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w)

        roll=roll*180/math.pi
        pitch=pitch*180/math.pi
        yaw=yaw*180/math.pi

        RPY = [round(roll, 4), round(pitch, 4), round(yaw, 4)]
        return translation, RPY


def findArucoMarker(img, draw=True):
    if (img is not None):
        imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rej = aruco.detectMarkers(imgGrey, dict, parameters=arucoParam)
    if draw:
        aruco.drawDetectedMarkers(img, bbox)
    return bbox


if __name__ == '__main__':

    cameraDist = np.float32(([[-0.25650196],[-0.0265314],[0.00340441],[-0.00166011],[0.13246781]]))
    cameraMatrix = np.float32([[538.43377448,0.00,338.3187159],[0.00,538.47465627,237.83438817],[0.00,0.00,1.00]])
#-0.0265314,0.00340441,-0.00166011,0.13246781


#538.43377448,0.00,338.3187159
# 0.00,538.47465627,237.83438817
# 0.00,0.00,1.00
    sample = 1

    try:
        while True:
            rt, frame = cap.read()
            edges = findArucoMarker(frame)
            if sample % 10 == 0:   
                translation, RPY = findXYZandRPY(edges, cameraDist=cameraDist,cameraMatrix=cameraMatrix)
                print(f"{sample}={{{translation[0]}, {translation[1]}, {translation[2]}}}, {{{RPY[0]}, {RPY[1]}, {RPY[2]}}}")
            if cv2.waitKey(1) == 13:       
                break

            cv2.imshow('aruco', frame)
            cv2.waitKey(5)

            sample += 1
    except Exception as e:
        pass

    cv2.destroyAllWindows()