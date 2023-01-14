# from curses.textpad import rectangle
import numpy as np
import cv2
import time

# local imports 
from gen_img_process import GenImgProcess

class Features(object):
	def __init__(self):
		# print("Feature Class Activated")
		self.class_lib_sides = {'remote': 4, 'suitcase': 4,
                         "laptop": 4, "keyboard": 4, "cell phone": 4, "book": 4}
		# image scale values
		self.kx = 1.414
		self.ky = 1.53
		self.gen_img_process = GenImgProcess()

	def R_z(self, angle):
		rad = np.radians(angle)
		return [[np.cos(rad), -np.sin(rad), 0], [np.sin(rad), np.cos(rad), 0], [0, 0, 1]]

	def pass_filter_variables(self, image, sel_lib, state_info):
		self.img = image
		# declare variables used to sort filters
		self.object_name = sel_lib['name']
		# used for area estimation
		self.overhead_area = sel_lib['over_A']
		# overhead resolution in form [height(y), width(x)]
		self.overhead_res = sel_lib['over_res']
		# declare variables used to postion arm relative to object
		self.xyz = state_info['xyz']
		self.angle4 = state_info['a4']
		self.angle5 = state_info['a5']
		self.ultrasonic_height = state_info['ultra_h']
		# slope of h/x & h/y
		self.mx = state_info['m_hx']
		self.my = state_info['m_hy']
		# scale values for approach
		self.scalex = self.xyz[2]*0.000282*self.mx
		self.scaley = self.xyz[2]*0.000306*self.my

	def search_val(self, elem):
		return elem[1]

	def find_move_angle(self, box, pxl_center):
		kx = self.kx
		ky = self.ky
		
		center = [pxl_center[0], pxl_center[1]]
		print(f"pixel center: {center}")
		points = [[box[0], box[1]],
                    [box[0], box[3]],
                    [box[2], box[1]],
                	[box[2], box[3]]]
		# define shift values for box
		shiftx = 640 - pxl_center[0]
		shifty = 360 - pxl_center[1]
		# shift all points so center of rectangle is in center
		# of screen
		for x in range(0, 4):
			points[x][0] = points[x][0] + shiftx
			points[x][1] = points[x][1] + shifty
		# verify that center is at center
		# print("center shifted x: ", (center[0] + shiftx) - 640)
		# print("center shifted y: ", -(center[1] + shifty) + 360)
		# convert points to cartesian plane
		for x in range(0, 4):
			points[x][0] = points[x][0] - 640
			points[x][1] = -points[x][1] + 360
		# print("Cartesian Points: ", points)
		# use first pair of cartesian coordinates to
		# define indexes for 3 different magnitudes
		mag_lst = []
		for i in range(1, 4):
			dx = points[i][0] - points[0][0]
			dy = points[i][1] - points[0][1]
			mag_lst.append([i, np.sqrt(dy**2+dx**2)])
		# order the magnitude list from largest magnitude to
		# smallest value
		mag_lst.sort(reverse=True, key=self.search_val)
		print("Ordered Magnitude List: ", mag_lst)
		# find magnitude of gripper grip_width
		wdx = ((points[mag_lst[2][0]][0]-points[0][0]))*self.scalex
		wdy = ((points[mag_lst[2][0]][1]-points[0][1]))*self.scaley
		grip_width = np.sqrt(wdx**2+wdy**2)
		print("Grip Width: ", grip_width)
		# print("Ordered Magnitude List: ", mag_lst)
		# midpoint of long side 1
		mid_x1 = points[0][0] + ((points[mag_lst[1][0]][0]-points[0][0])/2)
		mid_y1 = points[0][1] + ((points[mag_lst[1][0]][1]-points[0][1])/2)
		# print("Midpoint Corner(x,y),(x1,y1): ",[points[0][0],points[0][1]],[points[mag_lst[1][0]][0],points[mag_lst[1][0]][1]])
		# print("Midpoint (x,y)", [mid_x1,mid_y1])
		# midpoint of long side 2
		mid_x2 = points[mag_lst[2][0]][0] + ((points[mag_lst[0][0]][0]-points[mag_lst[2][0]][0])/2)
		mid_y2 = points[mag_lst[2][0]][1] + ((points[mag_lst[0][0]][1]-points[mag_lst[2][0]][1])/2)
		# makes unit vectos of the midpoints
		unit_scale1 = np.sqrt(mid_x1**2 + mid_y1**2)
		unit_scale2 = np.sqrt(mid_x2**2 + mid_y2**2)
		m1 = [mid_x1/unit_scale1, mid_y1/unit_scale1]
		m2 = [mid_x2/unit_scale2, mid_y2/unit_scale2]
		mdps = [m1, m2]
		print("These are the mid points: ", mdps)
		# find 2 angles based on the unit vector form of
		# the x and y values of each midpoint
		if mdps[0][0] > 0:
			theta1 = np.arccos(mdps[0][0])
			theta2 = np.arcsin(mdps[1][1])
			print("A")
		elif mdps[1][0] > 0:
			theta1 = np.arccos(mdps[1][0])
			theta2 = np.arcsin(mdps[0][1])
			print("B")
		else:
			theta1 = 0
			theta2 = 90
		# select smaller angle to return
		val = 0
		if abs(theta1) < abs(theta2):
			val = np.degrees(theta1)
		elif abs(theta1) > abs(theta2):
			val = np.degrees(theta2)
		else:
			print("theta1 must equal theta 2")
			val = np.degrees(theta2)

		print("Angle5 move value: ", val)
		return val, grip_width

	def scale_points(self, box):
		pts = []
		kx = self.kx
		ky = self.ky
		# print("self.xyz", self.xyz)
		h = self.xyz[2]-self.ultrasonic_height
		
		# find x,y,z for center of box
		mid_x = int(0.5*(box[2] - box[0]) + box[0])
		mid_y = int(0.5*(box[3] - box[1]) + box[1])
		# shiftx = 640 - pxl_center[0]
		# shifty = 360 - pxl_center[1]
		real_center = [[((mid_x-640)*self.scalex)], [((-mid_y+320)*self.scaley)], [h]]
		# print(f"Real Center -- feat: {real_center}")
		real_pts = [[((box[0]-640)*self.scalex), ((box[2]-640)*self.scalex), ((box[0]-640)*self.scalex), ((box[2]-640)*self.scalex)], 
					[((-box[1]+360)*self.scaley), ((-box[3]+360)*self.scaley), ((-box[3]+360)*self.scaley), ((-box[1]+360)*self.scaley)],
                    [h, h, h, h]]
		pxl_center = [mid_x, mid_y]
		pts.append(real_center)
		pts.append(real_pts)
		# finds amount of rotation to long side of angle
		move, grip_width = self.find_move_angle(box, pxl_center)
		return pts, move, grip_width

	def xyz_reframe(self, pts):
		# define homogenous transfer matricies
		h = self.xyz[2]-self.ultrasonic_height
		r_ab = self.R_z(self.angle4)
		r_bf = self.R_z(self.angle5)
		d_af = [[self.xyz[0]], [self.xyz[1]], [h]]
		r_af = np.dot(r_ab, r_bf)
		h_af = np.concatenate((r_af, d_af), 1)
		h_af = np.concatenate((h_af, [[0, 0, 0, 1]]), 0)
		center = [h_af[0][3], h_af[1][3], h_af[2][3]]
		print(f"Center -- feat: {center} ")
		# define no rotation matrix
		# r_fx = [[1,1,1],[1,1,1],[1,1,1]]
		# d_fc = [[pts[0][0]],[pts[0][1]],[0]]
		# h_fc = np.concatenate((r_fx,d_fc),1)
		# h_fc = np.concatenate((h_fc,[[0,0,0,1]]),0)
		# h_ac = np.dot(h_af,h_fc)
		# xyz = [h_ac[0][3],h_ac[1][3],h_ac[2][3]]
		# try adding displacment vectors to see difference from concatonation
		xyz = [float(pts[0][0]+center[0]), float(pts[0][1]+center[1]), float(center[2])]
		# print("Center Location:", xyz)
		rect = [[pts[1][0][0]+center[0], pts[1][0][1]+center[0], pts[1][0][2]+center[0], pts[1][0][3]+center[0]],
                    [pts[1][1][0]+center[1], pts[1][1][1]+center[1],
                    	pts[1][1][2]+center[1], pts[1][1][3]+center[1]],
                    [center[2], center[2], center[2], center[2]]]
		print(f"Rect -- Feat: {rect}")
		# print("Rectangles: ", np.matrix(rect))
		return xyz, rect

	def item_outline(self, passed_img):
		out_img, objects_info = self.gen_img_process.execute_model(passed_img)
		if len(objects_info) > 0:
			for items in objects_info.items():
				key, val = list(items)
				print(f"key: {key}, val: {val} -- features")
				# if key == self.object_name:
				found = True
				box = val
		else:
			found = False
			box = None

		return found, out_img, box

	def look_for_object(self, server, object_info, state_info):
		# cv2.imwrite('passed_img.jpg', passed_img)
		found, out_img, box = self.item_outline(self.img)
		# define "empty" varibles to avoid issues
		xyz = None
		grip_width = 0.1

		while not found:
			frame = server.request_camera("handcam")
			self.img = frame
			# load new information into class
			self.pass_filter_variables(frame, object_info, state_info)
			found, out_img, box = self.item_outline(self.img)
			time.sleep(1)
			print("Features Loop")
		if found:
			pts, move, grip_width = self.scale_points(box)
			xyz, rect = self.xyz_reframe(pts)

		return xyz, rect, move, grip_width, out_img
