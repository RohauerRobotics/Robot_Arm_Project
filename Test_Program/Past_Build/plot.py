import numpy as np

class Plot(object):
	def __init__(self, l1,l2,l3, h):
		self.l1 = l1
		self.l2 = l2
		self.l3 = l3
		self.h = h
		self.R_id = [[1, 1, 1], [1, 1, 1], [1, 1, 1]]
	
	def R_x(self, angle):
		rad = np.radians(angle)
		return [[1, 0, 0], [0, np.cos(rad), -np.sin(rad)], [0, np.sin(rad), np.cos(rad)]]
	
	def R_y(self, angle):
		rad = np.radians(angle)
		return [[np.cos(rad), 0, np.sin(rad)], [0, 1, 0], [-np.sin(rad), 0, np.cos(rad)]]
	
	def R_z(self, angle):
		rad = np.radians(angle)
		return [[np.cos(rad), -np.sin(rad), 0], [np.sin(rad), np.cos(rad), 0], [0, 0, 1]]
	
	def magnitude(self, vector):
		return np.sqrt(sum(pow(element, 2) for element in vector))
	
	def position(self, angles, grip_width):
		angle1 = angles[0]
		self.angle1 = np.radians(angle1)
		angle2 = angles[1]
		angle3 = angles[2]
		angle4 = angles[3]
		angle5 = angles[4]
		# rotation matrices
		r_0a = self.R_z(0)
		r_ab = self.R_z(angle4)
		self.r_ab = r_ab
		r_bc = self.R_y(angle1)
		r_cd = self.R_y(angle2)
		r_de = self.R_y(angle3)
		r_ef = self.R_z(angle5)
		# displacement matricies
		d_0a = [[0], [0], [self.h]]
		d_ab = [[0], [0], [0]]
		d_bc = [[np.sin(np.radians(angle1))*self.l1], [0], [np.cos(np.radians(angle1))*self.l1]]
		d_cd = [[np.sin(np.radians(angle2))*self.l2], [0], [np.cos(np.radians(angle2))*self.l2]]
		d_de = [[np.sin(np.radians(angle3))*self.l3], [0], [np.cos(np.radians(angle3))*self.l3]]
		# displacement matricies for claw
		d_ef = [[0], [0], [-self.l3/2]]
		d_f1 = [[grip_width/2], [0], [0]]
		d_f2 = [[-grip_width/2], [0], [0]]
		# homogenous transfer matricies
		#
		h_0a = np.concatenate((r_0a, d_0a), 1)
		h_0a = np.concatenate((h_0a, [[0, 0, 0, 1]]), 0)
		#
		h_ab = np.concatenate((r_ab, d_ab), 1)
		h_ab = np.concatenate((h_ab, [[0, 0, 0, 1]]), 0)
		#
		h_bc = np.concatenate((r_bc, d_bc), 1)
		h_bc = np.concatenate((h_bc, [[0, 0, 0, 1]]), 0)
		#
		h_cd = np.concatenate((r_cd, d_cd), 1)
		h_cd = np.concatenate((h_cd, [[0, 0, 0, 1]]), 0)
		#
		h_de = np.concatenate((r_de, d_de), 1)
		h_de = np.concatenate((h_de, [[0, 0, 0, 1]]), 0)
		h_ef = np.concatenate((r_ef, d_ef), 1)
		h_ef = np.concatenate((h_ef, [[0, 0, 0, 1]]), 0)
		#
		h_f1 = np.concatenate((self.R_id, d_f1), 1)
		h_f1 = np.concatenate((h_f1, [[0, 0, 0, 1]]), 0)
		#
		h_f2 = np.concatenate((self.R_id, d_f2), 1)
		h_f2 = np.concatenate((h_f2, [[0, 0, 0, 1]]), 0)
		#
		h_0b = np.dot(h_0a, h_ab)
		h_0c = np.dot(h_0b, h_bc)
		h_0d = np.dot(h_0c, h_cd)
		h_0e = np.dot(h_0d, h_de)
		# end effector test
		h_0f = np.dot(h_0e, h_ef)
		h_01 = np.dot(h_0f, h_f1)
		h_02 = np.dot(h_0f, h_f2)
		# print("h_af: ", h_af)
		# print('/n')
		# print("h_a1: ", h_a1)
		return [[[0, h_0b[0][3]], [0, h_0b[1][3]], [0, h_0b[2][3]]],
                [[h_0b[0][3], h_0c[0][3]], [h_0b[1][3], h_0c[1][3]], [h_0b[2][3], h_0c[2][3]]],
           		[[h_0c[0][3], h_0d[0][3]], [h_0c[1][3], h_0d[1][3]], [h_0c[2][3], h_0d[2][3]]], [
              [h_0d[0][3], h_0e[0][3]], [h_0d[1][3], h_0e[1][3]], [h_0d[2][3], h_0e[2][3]]],
              [[h_01[0][3], h_02[0][3]], [h_01[1][3], h_02[1][3]], [h_01[2][3], h_02[2][3]]]]
