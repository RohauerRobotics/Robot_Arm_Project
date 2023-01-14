import socket
import sys
import cv2
import pickle
import numpy as np
import struct ## new
import zlib
import time
import json

class Server(object):
	def __init__(self):
		pass
		# HOST=''
		# PORT=8485

		# sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		# print('Socket created')
		# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		# sock.bind((HOST,PORT))
		# print('Socket bind complete')
		# sock.listen(10)
		# print('Socket now listening')

		# self.conn, self.addr = sock.accept()
		# self.request_state()
		# self.send_zero_initialization()
		# # self.request_state()
		# camera = "handcam"
		# self.request_camera(camera)
		# self.send_instructions()
		# self.request_state()
		# self.request_state()

	def server_listen(self):
		HOST = ''
		PORT = 8485

		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		print('Socket created')
		sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		sock.bind((HOST, PORT))
		print('Socket bind complete')
		sock.listen(10)
		print('Socket now listening')

		self.conn, self.addr = sock.accept()

	def request_state(self):
		message = bytes("send_state", 'utf-8')
		self.conn.sendall(message)
		print("sent request")

		state_data = b""
		state_data = self.conn.recv(4096)
		state_dict = pickle.loads(state_data)
		print(state_dict)
		a1 = state_dict["a1"]
		print(f"angle 1: {a1}")

		recieved = bytes("recieved state", 'utf-8')
		self.conn.sendall(recieved)

	def request_camera(self, camera):
		message = "send_" + camera
		message = bytes(message, 'utf-8')
		self.conn.sendall(message)
		print("sent request")

		data = b""
		payload_size = struct.calcsize(">L")
		print("payload_size: {}".format(payload_size))
		while len(data) < payload_size:
			print("Recv: {}".format(len(data)))
			data += self.conn.recv(4096)

		print("Done Recv: {}".format(len(data)))
		packed_msg_size = data[:payload_size]
		data = data[payload_size:]
		msg_size = struct.unpack(">L", packed_msg_size)[0]
		print("msg_size: {}".format(msg_size))
		while len(data) < msg_size:
			data += self.conn.recv(4096)
		frame_data = data[:msg_size]
		data = data[msg_size:]

		frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
		frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

		recieved = bytes("recieved image", 'utf-8')
		self.conn.sendall(recieved)

		# cv2.imshow('ImageWindow',frame)
		# cv2.waitKey(0)
		return frame

	def send_instructions(self, instructions):
		message = bytes("movement_instructions", 'utf-8')
		self.conn.sendall(message)
		print("sent request for instructions")

		acknowledge = self.conn.recv(4096)
		print(str(acknowledge,'utf-8'))

		# instructions = {"a1":[1,2,2],"a2":[3,3,4],"a3":[5,6,6],"a4":[6,7,7],
		# "a5":[3,4,5],"a6":[4,5,6]}
		picked_instructions = pickle.dumps(instructions, 0)
		size = len(picked_instructions)
		self.conn.sendall(struct.pack(">L", size) + picked_instructions)

		acknowledge = self.conn.recv(4096)
		acknowledge.decode('utf-8')
		print(acknowledge)

	def send_zero_initialization(self):
		message = bytes("zero", 'utf-8')
		self.conn.sendall(message)
		print("sent request for zeroing")

		zero_data = b""
		zero_data = self.conn.recv(4096)
		zero_list = pickle.loads(zero_data)
		print(zero_list)
		# a1 = state_dict["a1"]
		# print(f"angle 1: {a1}")

		recieved = bytes("recieved initialization", 'utf-8')
		self.conn.sendall(recieved)


test = Server()
