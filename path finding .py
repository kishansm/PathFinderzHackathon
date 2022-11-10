import pygame, sys 
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from scipy.spatial.transform import Rotation
import cv2.aruco as aruco
from PIL import Image
import math
import server_comm as sc
import time
import cv2 
import numpy as np

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
# rt, frame1 = cap.read()
# cv2.imwrite("hello.jpg",frame1)

#aruco marker
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

# cv2.destroyAllWindows()















#path finding
class Pathfinder:
	def __init__(self,matrix):

		# setup
		self.matrix = matrix
		self.grid = Grid(matrix = matrix)
		self.select_surf = pygame.image.load('selection.png').convert_alpha()

		# pathfinding
		self.path = []

		# Roomba
		self.roomba = pygame.sprite.GroupSingle(Roomba(self.empty_path))

	def empty_path(self):
		self.path = []

	def draw_active_cell(self):
		mouse_pos = pygame.mouse.get_pos()
		# print(mouse_pos)
		row =  mouse_pos[1] // 32
		col =  mouse_pos[0] // 32
		current_cell_value = self.matrix[row][col]
		if current_cell_value == 1:
			rect = pygame.Rect((col * 32,row * 32),(32,32))
			screen.blit(self.select_surf,rect)

	def create_path(self):

		# start
		start_x, start_y = self.roomba.sprite.get_coord()
		start = self.grid.node(start_x,start_y)

		# end
		mouse_pos = pygame.mouse.get_pos()
		end_x,end_y =  mouse_pos[0] // 32, mouse_pos[1] // 32  
		end = self.grid.node(end_x,end_y) 

		# path
		finder = AStarFinder(diagonal_movement = DiagonalMovement.always)
		self.path,_ = finder.find_path(start,end,self.grid)
		self.grid.cleanup()
		self.roomba.sprite.set_path(self.path)

	def draw_path(self):
		if self.path:
			points = []
			for point in self.path:
				x = (point[0] * 32) + 16
				y = (point[1] * 32) + 16
				points.append((x,y))
			print(points)
			return points
			pygame.draw.lines(screen,'#4a4a4a',False,points,5)

	def print_points(self):
		points=self.draw_path()
		print(points)


	def update(self):
		self.draw_active_cell()
		
		self.draw_path()

		# roomba updating and drawing
		self.roomba.update()
		self.roomba.draw(screen)


class Roomba(pygame.sprite.Sprite):
	def __init__(self,empty_path):

		# basic
		super().__init__()
		self.image = pygame.image.load('roomba.png').convert_alpha()
		self.rect = self.image.get_rect(center = (40,550))

		# movement 
		self.pos = self.rect.center
		self.speed = 3
		self.direction = pygame.math.Vector2(0,0)

		# path
		self.path = []
		self.collision_rects = []
		self.empty_path = empty_path

	def get_coord(self):
		col = self.rect.centerx // 32
		row = self.rect.centery // 32
		return (col,row)

	def set_path(self,path):
		self.path = path
		self.create_collision_rects()
		self.get_direction()

	def create_collision_rects(self):
		if self.path:
			self.collision_rects = []
			for point in self.path:
				x = (point[0] * 32) + 16
				y = (point[1] * 32) + 16
				rect = pygame.Rect((x - 2,y - 2),(4,4))
				self.collision_rects.append(rect)

	def get_direction(self):
		if self.collision_rects:
			start = pygame.math.Vector2(self.pos)
			end = pygame.math.Vector2(self.collision_rects[0].center)
			self.direction = (end - start).normalize()
		else:
			self.direction = pygame.math.Vector2(0,0)
			self.path = []

	def check_collisions(self):
		if self.collision_rects:
			for rect in self.collision_rects:
				if rect.collidepoint(self.pos):
					del self.collision_rects[0]
					self.get_direction()
		else:
			self.empty_path()

	def update(self):
		self.pos += self.direction * self.speed
		self.check_collisions()
		self.rect.center = self.pos

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280,736))
clock = pygame.time.Clock()

# game setup

# bg_surf = pygame.image.load('darshan.jpeg').convert()
im = Image.open("darshan.jpeg")
width, height = im.size
newsize = (1280,736)
im1 = im.resize(newsize)
im1.save("resized_arena2.jpeg")
bg_surf = pygame.image.load("resized_arena2.jpeg").convert()
matrix = [
	[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
pathfinder = Pathfinder(matrix)

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			sys.exit()
		if event.type == pygame.MOUSEBUTTONDOWN:
			pathfinder.create_path()

	screen.blit(bg_surf,(0,0))
	pathfinder.update()

	pygame.display.update()
	clock.tick(60)