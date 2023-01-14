import matplotlib
matplotlib.use('TkAgg')
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
import time
import cv2
import PIL.Image, PIL.ImageTk

#
from multiprocessing import Lock, Process, Queue, current_process
import queue

# local imports
from simulation import Simulation

class UserInterface(object):
	def __init__(self):
		pass	

	# tkbase essential functions 

	def init_tkbase(self):
		# define root window
		self.root = tk.Tk()
		self.root.geometry('1460x760')
		self.root.title('Robot Arm Operation')
		self.root.grid_rowconfigure(0)
		self.root.grid_columnconfigure(2)
		self.root.resizable(0, 0)

		self.style = ("Times New Roman", 16)
		self.left_frame = tk.Frame(self.root, height=720, width=260)
		self.left_frame.config(highlightthickness = 1, highlightbackground = 'black')

		self.mid_frame = tk.Frame(self.root, height=720, width=480)
		self.mid_frame.config(highlightthickness = 1, highlightbackground = 'black')

		self.right_frame = tk.Frame(self.root, height=720, width=600)
		self.right_frame.config(highlightthickness = 1, highlightbackground = 'black')

		self.left_frame.grid(row=0,column=0, padx=20, pady=20, sticky="ns")
		self.mid_frame.grid(row=0,column=1, padx=20, pady=20, sticky="nsew")
		self.right_frame.grid(row=0,column=2, padx=20, pady=20, sticky="ns")
		# configure leftmost frame
		self.left_frame.grid_rowconfigure(2)
		self.left_frame.grid_columnconfigure(0)

		self.command_title = tk.Label(
			self.left_frame, text="Control Commands", font=self.style)

		self.button_frame = tk.Frame(self.left_frame, height=420, width=260)
		# self.button_frame.config(highlightthickness=1, highlightbackground='black')

		self.lower_button_frame = tk.Frame(self.left_frame, height=120, width=260)
		# self.lower_button_frame.config(
			# highlightthickness=1, highlightbackground='black')

		self.command_title.grid(row=0, column=0, padx=3, pady=20, sticky="n")
		self.button_frame.grid(row=1, column=0, padx=0, pady=20, sticky="ew")
		self.lower_button_frame.grid(row=2, column=0, padx=0, pady=20, sticky="s")
		# configure middle frame
		self.mid_frame.grid_rowconfigure(1)
		self.mid_frame.grid_columnconfigure(0)

		self.img_title = tk.Label(self.mid_frame, text="Camera View", font=self.style)

		self.img_frame = tk.Frame(self.mid_frame, height=480, width=480)
		# self.img_frame.config(highlightthickness = 1, highlightbackground = 'black')

		self.img_title.grid(row=0,column=0, padx=3, pady=20, sticky="n")
		self.img_frame.grid(row=1,column=0, padx=1, pady=20, sticky="nsew")
		# configure middle frame
		self.right_frame.grid_rowconfigure(1)
		self.right_frame.grid_columnconfigure(0)

		self.sim_title = tk.Label(self.right_frame, text="Simulation View", font=self.style)

		self.sim_frame = tk.Frame(self.right_frame, height=500, width=600)
		# self.sim_frame.config(highlightthickness = 1, highlightbackground = 'black')

		self.sim_title.grid(row=0,column=0, padx=3, pady=20, sticky="n")
		self.sim_frame.grid(row=1,column=0, padx=0, pady=20, sticky="nsew")

	def init_simulation(self):
		self.sim = Simulation()
		self.sim.fig = plt.figure()
		self.sim.ax = Axes3D(self.sim.fig)
		self.sim.ani_running = False
		self.sim.init_sim_frame(self.sim_frame)

	def init_buttons_style_A(self, button_lib):
		# make 3 rows for this button style
		self.button_frame.grid_columnconfigure(0)
		self.button_frame.grid_rowconfigure(3)
		common_padx = 50

		if (button_lib["Label"]["text"] != None):
			self.button_titleA = tk.Label(self.button_frame, text=button_lib["Label"]["text"],font=self.style)
			self.button_titleA.grid(row=0,column=0, padx=common_padx, pady=20, sticky="n")

		if (button_lib["A1"]["text"] != None) & (button_lib["A1"]["function"] != None):
			self.button_A1 = tk.Button(self.button_frame, text=button_lib["A1"]["text"],
					command=lambda:(button_lib["A1"]["function"])(button_lib["A1"]["kwargs"]), font=self.style)
			self.button_A1.grid(row=1,column=0, padx=common_padx, pady=20, sticky="ew")

		if (button_lib["B1"]["text"] != None) & (button_lib["B1"]["function"] != None):
			self.button_B1 = tk.Button(self.button_frame, text=button_lib["B1"]["text"],
					command=lambda:(button_lib["B1"]["function"])(button_lib["B1"]["kwargs"]), font=self.style)
			self.button_B1.grid(row=2,column=0, padx=common_padx, pady=20, sticky="ew")

		if (button_lib["C1"]["text"] != None) & (button_lib["C1"]["function"] != None):
			self.button_C1 = tk.Button(self.button_frame, text=button_lib["C1"]["text"],
					command=lambda:(button_lib["C1"]["function"])(button_lib["C1"]["kwargs"]), font=self.style)
			self.button_C1.grid(row=3,column=0, padx=common_padx, pady=20, sticky="s")

	def init_buttons_style_B(self, button_lib):
		self.button_frame.grid_columnconfigure(2, weight=1)
		self.button_frame.grid_rowconfigure(2)
		common_padx = 32.5

		if (button_lib["Label"]["text"] != None):
			self.button_titleB = tk.Label(self.button_frame, text=button_lib["Label"]["text"],font=self.style)
			self.button_titleB.grid(row=0,column=0,columnspan=3, padx=10, pady=20, sticky="ew")

		if (button_lib["A1"]["text"] != None) & (button_lib["A1"]["function"] != None):
			self.button_A1 = tk.Button(self.button_frame, text=button_lib["A1"]["text"],
					command=lambda:(button_lib["A1"]["function"])(button_lib["A1"]["kwargs"]), font=self.style)
			self.button_A1.grid(row=1,column=0, padx=common_padx, pady=20, sticky="e")

		if (button_lib["A2"]["text"] != None) & (button_lib["A2"]["function"] != None):
			self.button_A2 = tk.Button(self.button_frame, text=button_lib["A2"]["text"],
					command=lambda:(button_lib["A2"]["function"])(button_lib["A2"]["kwargs"]), font=self.style)
			self.button_A2.grid(row=1,column=1, padx=5, pady=20, sticky="ns")

		if (button_lib["A3"]["text"] != None) & (button_lib["A3"]["function"] != None):
			self.button_A3 = tk.Button(self.button_frame, text=button_lib["A3"]["text"],
					command=lambda:(button_lib["A3"]["function"])(button_lib["A3"]["kwargs"]), font=self.style)
			self.button_A3.grid(row=1,column=2, padx=common_padx, pady=20, sticky="w")

		if (button_lib["B1"]["text"] != None) & (button_lib["B1"]["function"] != None):
			self.button_B1 = tk.Button(self.button_frame, text=button_lib["B1"]["text"],
					command=lambda:(button_lib["B1"]["function"])(button_lib["B1"]["kwargs"]), font=self.style)
			self.button_B1.grid(row=2,column=0, padx=5, pady=20, sticky="e")

		if (button_lib["B2"]["text"] != None) & (button_lib["B2"]["function"] != None):
			self.button_B2 = tk.Button(self.button_frame, text=button_lib["B2"]["text"],
					command=lambda:(button_lib["B2"]["function"])(button_lib["B2"]["kwargs"]), font=self.style)
			self.button_B2.grid(row=2,column=2, padx=5, pady=20, sticky="w")

	def init_lower_button(self, button_lib):
		self.lower_button_frame.grid_columnconfigure(0)
		self.lower_button_frame.grid_rowconfigure(1)


		if (button_lib["Label"]["text"] != None):
			self.button_title = tk.Label(self.button_frame, text=button_lib["Label"]["text"],font=self.style)
			self.button_title.grid(row=0,column=0, padx=50, pady=20, sticky="n")

		if (button_lib["A1"]["text"] != None) & (button_lib["A1"]["function"] != None):
			self.lower_button = tk.Button(self.lower_button_frame, text=button_lib["A1"]["text"],
											command=lambda:(button_lib["A1"]["function"])(button_lib["A1"]["kwargs"]), font=self.style)
			self.lower_button.grid(row=1,column=0, padx=50, pady=40, sticky="n")
		# print("Assigned Simulation Button")

	def display_image(self, frame):
		self.img_frame.grid_rowconfigure(0)
		self.img_frame.grid_columnconfigure(0)

		self.image_canvas = tk.Canvas(self.img_frame, width=480, height=320)
		self.image_canvas.grid(row=0, column=0, pady=100)


		img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		img = cv2.resize(img,(480,320))

		# print(f"cv2 img: {img}")
		pil_img = PIL.Image.fromarray(img)
		self.photo = PIL.ImageTk.PhotoImage(image=pil_img)
		self.image_canvas.create_image(0, 0, image=self.photo, anchor="nw")

	def update_image(self, frame):
		img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		img = cv2.resize(img, (480, 320))

		# print(f"cv2 img: {img}")
		pil_img = PIL.Image.fromarray(img)
		self.photo = PIL.ImageTk.PhotoImage(image=pil_img)
		self.image_canvas.create_image(0, 0, image=self.photo, anchor="nw")
	
	def remove_frame_items(self, frame):
		for widgets in frame.winfo_children():
			widgets.destroy()
	
	# multiprocessing button functions 
	# logic pathways built in and shared with arm_compute

	def run_user_interface(self, command, data_pass, progress):
		print("Running User Interface")
		self.init_tkbase()
		self.init_simulation()
		self.make_button_libs(command, data_pass, progress)
		self.init_buttons_style_A(self.init_button_lib)

		self.root.mainloop()

	def make_button_libs(self, command, data_pass, progress):
		queue_list = [command, data_pass, progress]
		# intialization libraries
		self.init_button_lib = {"Label": {"text": None},
                          "A1": {"text": "Initialize", "function": self.init_command, "kwargs": queue_list},
                          "B1": {"text": None, "function": None, "kwargs": None},
                          "C1": {"text": None, "function": None, "kwargs": None}
                          }

		self.item_sel_lib = {"Label": {"text": "Select Object \n to Move"},
                       "A1": {"text": None, "function": self.item_select, "kwargs": None},
                       "B1": {"text": None, "function": self.item_select, "kwargs": None},
                       "C1": {"text": None, "function": self.item_select, "kwargs": None}
                       }
		self.num_approaches_lib = {"Label": {"text": "Select Number \n of Approaches"}, "A1": {"text": "2", "function": self.num_approaches, "kwargs": "Number of selections: 2"},
                             "A2": {"text": "3", "function": self.num_approaches, "kwargs": "Number of selections: 3"},
                             "A3": {"text": "4", "function": self.num_approaches, "kwargs": "Number of selections: 4"},
                             "B1": {"text": "5", "function": self.num_approaches, "kwargs": "Number of selections: 5"},
                             "B2": {"text": "6", "function": self.num_approaches, "kwargs": "Number of selections: 6"}}

		# recapture image initialization library
		self.recap_img_lib = {"Label": {"text": None},
                        "A1": {"text": "Recapture \n Image", "function": self.refresh_objects, "kwargs": None}
                        }

	def init_command(self, queue_list):
		print(queue_list)
		command, data_pass, progress = queue_list[0], queue_list[1], queue_list[2]
		command.put("initialize")

		while data_pass.empty():
			pass

		frame = data_pass.get()
		print("Got Image -- UI")

		prog_empty = progress.empty()

		while progress.empty():
			pass

		prog = progress.get()

		# print("Got progress")
		# print("Image: \n", frame)

		self.display_image(frame)
		time.sleep(0.05)
		self.remove_frame_items(self.button_frame)
		time.sleep(0.05)
		# create kwargs for item select function
		kwarg_list = [command, data_pass, progress, prog]
		for idx in ["A1", "B1", "C1"]:
			item = prog["cam_objects"][idx]
			box = prog["cam_object_box"][idx]
			kwarg_list.append(item)
			kwarg_list.append(box)
			self.item_sel_lib[idx]["text"] = item
			self.item_sel_lib[idx]["kwargs"] = kwarg_list
			kwarg_list = [command, data_pass, progress, prog]

		self.init_buttons_style_A(self.item_sel_lib)
		# make refresh objects button
		# kwarg_list.append(False)
		self.recap_img_lib["A1"]["kwargs"] = kwarg_list
		self.init_lower_button(self.recap_img_lib)
	
	def item_select(self, kwarg_list):
		command, data_pass, progress = kwarg_list[0], kwarg_list[1], kwarg_list[2]
		# print(command, data_pass, progress)
		prog, item, box = kwarg_list[3], kwarg_list[4], kwarg_list[5]
		# print(prog, item, dest)

		if prog["object_sel"]["name"] == None:
			prog["object_sel"]["name"] = item
			prog["object_sel"]["box"] = box
			# print("Check if data_pass is empty before request: ", data_pass.empty())

			command.put("get_overheadcam")
			print("put get overheadcam -- UI")
			progress.put(prog)
			while data_pass.empty():
				pass
			frame = data_pass.get()
			# cv2.imshow("frame", frame)
			# cv2.waitKey(0)

			print("Got Image -- item select")
			time.sleep(0.2)
			while progress.empty():
				pass
			prog = progress.get()
			print("Got Prog -- item select")
			# print(prog)
			# print("img_frame data",self.img_frame.winfo_children())
			# self.remove_frame_items(self.img_frame)
			# time.sleep(5)
			self.update_image(frame)
			# create kwargs for item select function
			kwarg_list = [command, data_pass, progress, prog]
			
			print(f"prog lib -- UI item sel dest= False:{prog}")

			self.item_sel_lib["Label"]["text"] = "Select \n destination"
			for idx in ["A1", "B1", "C1"]:
				item = prog["cam_objects"][idx]
				if (item != prog["object_sel"]["name"]):
					print(idx,": ", item)
					box = prog["cam_object_box"][idx]
					kwarg_list.append(item)
					kwarg_list.append(box)
					self.item_sel_lib[idx]["text"] = item
					self.item_sel_lib[idx]["kwargs"] = kwarg_list
					kwarg_list = [command, data_pass, progress, prog]
				else:
					self.item_sel_lib[idx]["text"] = None

			self.remove_frame_items(self.button_frame)
			self.remove_frame_items(self.lower_button_frame)
			time.sleep(1.5)
			# kwarg_list.append(True)
			self.recap_img_lib["kwargs"] = kwarg_list
			self.init_lower_button(self.recap_img_lib)
			self.init_buttons_style_A(self.item_sel_lib)

		elif prog["object_sel"]["name"] != None:
			self.remove_frame_items(self.lower_button_frame)
			prog["destination_sel"]["name"] = item
			prog["destination_sel"]["box"] = box
			print(f"Destination progress: {prog}")
			command.put("move_to")
			progress.put(prog)
			time.sleep(1)
			# wait for new prog lib
			while progress.empty():
				pass
			prog = progress.get()
			print("New Prog: ", prog)

			self.remove_frame_items(self.button_frame)
			time.sleep(0.05)
			kwarg_list = [command, data_pass, progress, prog]
			count = 2
			for idx in ["A1", "A2", "A3", "B1", "B2"]:
				kwarg_list.append(str(count))
				self.num_approaches_lib[idx]["kwargs"] = kwarg_list
				kwarg_list = [command, data_pass, progress, prog]
				# print(count)
				count += 1
			self.init_buttons_style_B(self.num_approaches_lib)

	def num_approaches(self, kwarg_list):
		print("Kwarg List: ", kwarg_list)
		command, data_pass, progress = kwarg_list[0], kwarg_list[1], kwarg_list[2]
		prog, count = kwarg_list[3], kwarg_list[4]
		print("num approaches: ", prog, count)
		prog["num_approaches"] = count
		progress.put(prog)
		print(f"Put Num Approaches -- UI: {count}")
		self.remove_frame_items(self.button_frame)
		while data_pass.empty():
			pass
		data_pkg = data_pass.get()
		suite, static_pkg = data_pkg[0], data_pkg[1]
		self.sim.call_animation(static_pkg, suite)
		iter = 0
		while iter < int(count):
			while data_pass.empty():
				pass
			display_img = data_pass.get()
			self.update_image(display_img)
			
			while data_pass.empty():
				pass
			data_pkg = data_pass.get()
			suite, static_pkg = data_pkg[0], data_pkg[1]
			self.sim.stop_animation()
			self.sim.call_animation(static_pkg, suite)
			iter += 1
			print(f"Count -- UI : {iter}")
	
	def refresh_objects(self, kwarg_list):
		command, data_pass, progress = kwarg_list[0], kwarg_list[1], kwarg_list[2]
		prog = kwarg_list[3]
		# if object select frame needs to be refreshed
		# print(f"Refresh Button Called, dest: {dest}")
		if prog["object_sel"]["name"] == None:
			# clear button items
			self.remove_frame_items(self.button_frame)
			command.put("get_overheadcam")
			progress.put(prog)

			while data_pass.empty():
				pass

			frame = data_pass.get()

			while progress.empty():
				pass

			prog = progress.get()

			# update image
			self.update_image(frame)

			# make new object select lib
			kwarg_list = [command, data_pass, progress, prog]
			for idx in ["A1", "B1", "C1"]:
				item = prog["cam_objects"][idx]
				box = prog["cam_object_box"][idx]
				kwarg_list.append(item)
				kwarg_list.append(box)
				self.item_sel_lib[idx]["text"] = item
				self.item_sel_lib[idx]["kwargs"] = kwarg_list
				kwarg_list = [command, data_pass, progress, prog]

			self.init_buttons_style_A(self.item_sel_lib)

		elif prog["object_sel"]["name"] != None:
			# clear button items
			self.remove_frame_items(self.button_frame)
			command.put("get_overheadcam")
			progress.put(prog)

			while data_pass.empty():
				pass

			frame = data_pass.get()

			while progress.empty():
				pass

			prog = progress.get()

			# update image
			self.update_image(frame)

			# make new object select lib
			kwarg_list = [command, data_pass, progress, prog]
			self.item_sel_lib["Label"]["text"] = "Select \n destination"
			for idx in ["A1", "B1", "C1"]:
				item = prog["cam_objects"][idx]
				if item != prog["object_sel"]["name"]:
					kwarg_list.append(item)
					box = prog["cam_object_box"][idx]
					kwarg_list.append(box)
					self.item_sel_lib[idx]["text"] = item
					self.item_sel_lib[idx]["kwargs"] = kwarg_list
					kwarg_list = [command, data_pass, progress, prog]
				else:
					self.item_sel_lib[idx]["text"] = None

			self.init_buttons_style_A(self.item_sel_lib)

# test = TkBase()
