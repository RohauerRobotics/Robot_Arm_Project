# arm_compute operates and connects all of the aspects of the robot arm control
from multiprocessing import Lock, Process, Queue, current_process
import queue
import time

# local imports
from server import Server
from user_interface import UserInterface
from gen_img_process import GenImgProcess
from plot_operator import PlotOperator

class Operator(object):
	def __init__(self):
		self.server = Server()
		self.user_interface = UserInterface()

		# initialization request and pass queues
		command = Queue()
		# image request and pass queues
		data_pass = Queue()
		progress = Queue()

		# thread for operating server connection, image processing
		# and simulations
		operate_p = Process(target=self.operate, args=(
			command, data_pass, progress))
		operate_p.start()

		# thread for running user interface
		user_interface_p = Process(
			target=self.user_interface.run_user_interface, args=(command, data_pass, progress))
		user_interface_p.start()

		processes = [operate_p, user_interface_p]

		for p in processes:
			p.join()

	def make_init_lib(self, entry):
		init_prog_lib = {"process":entry,
						"cam_objects":{"A1":None,"B1":None, "C1":None},
                		"cam_object_box": {"A1": None, "B1": None, "C1": None},
						"object_sel":{"name":None,"box":None},
            		   "destination_sel": {"name": None, "box": None}}
		return init_prog_lib

	def make_grab_obj_lib(self):
		grab_obj_lib = {"process": "grab_obj", "num_approaches":None,"confirm_grasp":None}
		return grab_obj_lib

	def operate(self, command, data_pass, progress):
		# set up server listening 
		self.server.server_listen()

		# setup class function for general image processing
		self.gen_process = GenImgProcess()
		entry = None
		while True:

			if not command.empty():
				entry = command.get()

			if entry == "initialize":
				entry = self.initialize(progress, data_pass, entry)

			elif entry == "get_overheadcam":
				prog = self.pause_queue(progress)

				if prog["process"] == "initialize":
					entry = self.overhead_op(progress, data_pass, prog)

			elif entry == "move_to":
				prog = self.pause_queue(progress)

				if prog["process"] == "initialize":
					obj_lib = prog["object_sel"]
					prog = self.make_grab_obj_lib()
					progress.put(prog)
					time.sleep(2)
					prog = self.pause_queue(progress)
					print(f"Moving to Object -- AC: {obj_lib}")
					
					# begin simulation approach
					state_vals = {'a1': 0, 'a2': 0, 'a3': 0, 'a4': 0, 'a5': 0, 'a6': 0, 'ultra': 4.76}
					init_vals = {'a1': 1.64, 'a2': 2, 'a3': 2, 'a4': 1, 'a5': 1, 'a6': 1}
					self.plot_op = PlotOperator(state_vals, init_vals)
					queues = [data_pass, progress]
					self.plot_op.pickup_object(queues, self.server, prog, obj_lib)
					entry = None

	def pause_queue(self, queue):
		while queue.empty():
			pass
		ret = queue.get()
		return ret

	def assign_screen_items(self, prog, obj_lib):
		key_list = []
		for key in obj_lib.keys():
			key_list.append(key)

		print(f"Length of keys: {len(key_list)}")
		print(f"Object Keys: {key_list}")
		if len(key_list) <= 2:
			idx_len = len(obj_lib.keys())
		else:
			idx_len = 2
		cam_obj_idx = ["A1", "B1", "C1"]
		for i in range(0, idx_len):
			prog["cam_objects"][cam_obj_idx[i]] = key_list[i]
			prog["cam_object_box"][cam_obj_idx[i]] = obj_lib[key_list[i]]
		
		return prog

	def initialize(self, progress, data_pass, entry):
		frame = self.server.request_camera("overheadcam")
		# add function to get ojects in frame and
		# frame with boxes around object
		out_img, obj_lib = self.gen_process.execute_model(frame)

		print(f"Object Lib --AC: {obj_lib}")
		prog = self.make_init_lib(entry)
		# print("Progress: \n", prog)
		data_pass.put(out_img)

		# reassign items for "objects on screen"
		prog = self.assign_screen_items(prog, obj_lib)
		progress.put(prog)
		# time.sleep(.1)
		print("Put progress --  AC: ", not progress.empty())
		entry = None

		return entry

	def overhead_op(self, progress, data_pass, prog):
		frame = self.server.request_camera("overheadcam")
		# add function to get ojects in frame and
		# frame with boxes around object
		out_img, obj_lib = self.gen_process.execute_model(frame)

		print(f"Object Lib --AC: {obj_lib}")
		data_pass.put(out_img)

		# reassign items for "objects on screen"
		prog = self.assign_screen_items(prog, obj_lib)

		print("Passed Prog: -- AC", prog)
		progress.put(prog)
		entry = None

		return entry 

if __name__ == "__main__":
	test = Operator()
