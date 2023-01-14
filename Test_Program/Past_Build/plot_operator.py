# path plotter returns an animation suite and stepper suite
# inputs are arm's current state, stepper calibration values, and overhead or hand image results
import numpy as np
import time

# local imports 
from path import Path
from plot import Plot
from features import Features

class PlotOperator(object):
	def __init__(self, state_vals, init_vals):
		# what state_value input looks like:
		# state_vals = {'a1': 73.3447265625, 'a2': None, 'a3': None, 
		# 				'a4': None, 'a5': None, 'a6': None, 'ultra': 4.759454727172852}
		
		# what initialization values look like:
		# init_vals = {'a1': 1.6429265694694095, 'a2': None, 'a3': None, 'a4': None, 'a5': None, 'a6': None}
		# define arm segment lengths
		self.l1 = 0.45
		self.l2 = 0.45
		self.l3 = 0.1

		# define height of stepper motor off of the ground
		self.h = 0.15

		state_vals.pop("ultra")
		print(state_vals)

		# define stepper settings
		self.micro_steps = 200
		self.w_max = np.pi
		self.accel = np.pi/8

		# scale factors
		self.kx = 1.4464
		self.ky = 1.5403
		self.metersperangle = 0.005

		# make initial angle list and angle adjustment list
		self.angle_adj = []
		self.initial = []
		for key in state_vals.keys():
			self.initial.append(state_vals[key])
			self.angle_adj.append(init_vals[key])
		print(f"angle adj: {self.angle_adj}")
		print(f"init angles: {self.initial}")

		# stand in values
		self.initial[4] = 90
		self.initial[5] = 20

		# define class variables
		self.calc = Calc(self.l1, self.l2, self.l3, self.h)
		self.plot = Plot(self.l1, self.l2, self.l3, self.h)

		print(f"Angle Adjustment Vals: {self.angle_adj}")

	def pickup_object(self, queues, server, prog, sel_lib):
		# define class queue objects
		self.data_pass, self.progress = queues[0], queues[1]
		self.server_op = server
		self.prog = prog
		suite, static_pkg, long_delay, short_delay, object_info, state_info, last_angles = self.first_approach(sel_lib)

		# post initial simulation data to User Interface
		data_pkg = [suite, static_pkg]
		self.data_pass.put(data_pkg)

		# post movement instructions to client
		self.server_op.send_instructions([long_delay, short_delay])

		grip = self.iterative_approach(int(prog["num_approaches"]), suite, static_pkg, object_info, state_info, last_angles)

	def first_approach(self, sel_lib):
		# initialize path class variable
		path_op = Path(self.initial, self.angle_adj, self.l1, self.l2, self.l3)
		suite = []
		# get xy box from selection library
		xy_box = sel_lib["box"]

		# xyz is a list of the end effector position over goal
		xyz = self.calc.two_to_three_dims(xy_box)
		print(f"Goal Position (m): {xyz}")
		path, long_delay, short_delay, can_reach = path_op.find_path_to(xyz)

		# arranges static data for plotting box on in simulation
		pts = self.calc.make_points(xy_box, xyz)

		self.static_pkg = {'goals': pts[0], 'outline': pts[1]}
		
		if can_reach:
			for x in range(0, len(path[0])):
				post = self.plot.position([path[0][x], path[1][x], path[2][x], path[3][x], path[4][x]], self.metersperangle*self.initial[5])
				suite.append(post)

			# for debugging only -- get info from server in actual run for iterative_approach
			state_info =  {"a1": path[0][-1],"a2":path[1][-1],"a3":path[2][-1],"a4":path[3][-1],"a5":path[4][-1],"a6":self.initial[5]}
			state_info["xyz"] = xyz
			state_info['m_hx'] = 4
			state_info['m_hy'] = 4
			state_info['ultra_h'] = xyz[2] - 0.01
			last_angles = [path[0][-1], path[1][-1], path[2][-1], path[3][-1], path[4][-1], self.initial[5]]
		else:
			print("Couldn't Plot Path")

		object_info = {"name":sel_lib["name"], "over_A":(xy_box[0]-xy_box[2])*(xy_box[1]-xy_box[3]),"over_res":[720, 1280]}
		
		return suite, self.static_pkg, long_delay, short_delay, object_info, state_info, last_angles
		
	def iterative_approach(self, num_updates, suite, static_pkg, object_info, state_info, last_angles):
		# plotter = Plot(self.path.values['len'])
		# suite = []
		feat = Features()
		print("Initialized Features")
		last_move = 0
		for i in range(0, num_updates):
			# polls hand camera for image
			frame = self.server_op.request_camera("handcam")
			
			# use feature class to find center of object, angle, ect
			feat.pass_filter_variables(frame, object_info, state_info)
			new_center, rect, move, grip_width, display_img = feat.look_for_object(self.server_op, object_info, state_info)
			print(f"New Center: {new_center}")
			# post handcam image to user interface
			self.data_pass.put(display_img)
			time.sleep(1)
			actual_move = move - last_move
			last_move = move
			if (new_center != None):
				static_pkg, goal_xyz = self.calc.arrange_static(rect, new_center, state_info, static_pkg, i, num_updates)
				path_op = Path(last_angles, self.angle_adj, self.l1, self.l2, self.l3)
				path1, long_delay, short_delay, can_reach = path_op.find_path_to(goal_xyz, change_wrist_angle=True)
				# will grasp item if count is final
				if i == (num_updates-1):
					applied_width = grip_width
					angle6 = self.metersperangle/applied_width
				else:
					applied_width = self.metersperangle*self.initial[5]
					angle6 = self.initial[5]

				if can_reach:
					for x in range(0, len(path1[0])):
						post = self.plot.position(
							[path1[0][x], path1[1][x], path1[2][x], path1[3][x], path1[4][x]], applied_width)
						# post.append(pt)
						suite.append(post)
					# post data to the simulation
					data_pkg = [suite, static_pkg]
					self.data_pass.put(data_pkg)
					# determine xyz of the end effector
					last_angles = [path1[0][-1], path1[1][-1], path1[2]
                                            [-1], path1[3][-1], path1[4][-1], angle6]
					xyz = goal_xyz
					state_info =  {"a1": path1[0][-1],"a2":path1[1][-1],"a3":path1[2][-1],"a4":path1[3][-1],"a5":path1[4][-1],"a6":self.initial[5]}
					state_info["xyz"] = xyz
					state_info['m_hx'] = 4
					state_info['m_hy'] = 4
					state_info['ultra_h'] = xyz[2] - 0.01
				else:
					print("F")
		#  ------------------------------
		print("applied width:", applied_width)
		return applied_width

class Calc(object):
	def __init__(self, l1, l2, l3, h):
		self.l1 = l1
		self.l2 = l2
		self.l3 = l3
		self.h = h

		# scale values for overhead camera
		# kx = 1.244
		# ky = 1.288
		self.kx = 1.4464
		self.ky = 1.5403

	def two_to_three_dims(self, xy_box):
		# convert box coordinates into real world coordinates
		w_center = self.calc_world_pos(xy_box)

		# calculates distance from orgin to item 
		flat_coord = np.sqrt((w_center[0]*w_center[0]) + (w_center[1]*w_center[1]))
		tot_length = (self.l1 + self.l2)

		# based on Pythogorean Triangle
		# a^2 = c^2 - b^2
		max_w = np.sqrt((tot_length**2-self.l3**2))

		# technically not true if object is below y=0, but this can be adjusted for later
		if flat_coord > max_w:
			print("Cannot Reach")
			end_effector = None

		elif flat_coord < max_w:
			for x in range(0, 90):
				w = np.cos(np.radians(90-x))*tot_length
				h = np.sin(np.radians(90-x))*tot_length
				if (tot_length) >= (np.sqrt(flat_coord**2 + h**2)):
					w_center[2] = h - self.h
					end_effector = [w_center[0], w_center[1], w_center[2]]
					break
				else:
					end_effector = None
		else:
			end_effector = None
		print("xyz is: ", end_effector)

		return end_effector

	def calc_world_pos(self, xy_box):
		# based on
		center = [abs((xy_box[2]-xy_box[0])/2)+xy_box[0]-640, -
                    (abs((xy_box[3]-xy_box[1])/2)+xy_box[1])+360, 0]
		# print('\n Object Cented at:(in pxls)',center)
		w_center = [(center[0]*self.kx)/1000, (center[1]*self.ky)/1000, 0]
		print('/n Object Cented at:(in meters)', w_center)
		return w_center

	def make_points(self, xy_box, xyz):
		pts = []
		# xfyfzf = self.two_three_dim(travel[2][1])
		pts.append([[xyz[0]],
                    [xyz[1]],
                    [xyz[2]+self.h-(self.l3/2)]])

		pts.append([[((xy_box[0]-640)*self.kx)/1000, ((xy_box[0]-640)*self.kx)/1000, ((xy_box[2]-640)*self.kx)/1000, ((xy_box[2]-640)*self.kx)/1000],
                    [((-xy_box[1]+360)*self.ky)/1000, ((-xy_box[3]+360)*self.ky)/1000,
                     ((-xy_box[1]+360)*self.ky)/1000, ((-xy_box[3]+360)*self.ky)/1000],
                    [0, 0, 0, 0]])
		return pts

	def arrange_static(self, rect, new_center, state_info, static_pkg, i, num_updates):
		distance = state_info['ultra_h']
		print("divisor: ", num_updates-i)
		z_val = (distance - ((distance)/(num_updates-i)))

		# make sure it doesn't go through ground
		if z_val < (self.l3-self.h):
			z_val = self.l3-self.h
		else:
			pass
		static_pkg["goals"][0].append(new_center[0])
		static_pkg["goals"][1].append(new_center[1])
		static_pkg["goals"][2].append(z_val+self.h-(self.l3/2))
		static_pkg["outline"][0] = rect[0]
		static_pkg["outline"][1] = rect[1]
		static_pkg["outline"][2] = rect[2]

		# define end effector goal position
		goal_xyz = [new_center[0], new_center[1], z_val]
		return static_pkg, goal_xyz
