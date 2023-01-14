from mimetypes import init
import numpy as np

class Path(object):
	def __init__(self, inital_angles, angle_adj, l1, l2, l3):
		self.l1 = l1
		self.l2 = l2
		self.l3 = l3
		self.inital_angles = inital_angles
		self.angle_adj = angle_adj

		self.micro_steps = 200
		self.w_max = np.pi
		self.accel = np.pi/8

	def find_path_to(self, xyz, change_wrist_angle=False):
		if xyz != None:
			# final is 
			end_angles, can_reach = self.inverse_kinematics(xyz)
			# used for iterative approach
			if change_wrist_angle:
				end_angles.append(self.inital_angles[4] - end_angles[3])
			else:
				end_angles.append(self.inital_angles[4])
		# if no angles are given the arm cannot reach goal
		elif xyz == None:
			can_reach = False
		
		if can_reach:
			self.accel, self.ani_accel, travel_time = self.acceleration_path(
				self.inital_angles, end_angles, self.angle_adj)
			path, divisions = self.animation_path(self.inital_angles,
				end_angles, self.accel, self.ani_accel, travel_time, self.angle_adj)
			delay_times, short_delay = self.stepper_path(self.step_path)
		else:
			path = None

		return path, delay_times, short_delay, can_reach

	def inverse_kinematics(self, end_e):
		# assign the x,y,z coordinates of the end
		# effector goal position shorter variable names
		x = end_e[0]
		y = end_e[1]
		z = end_e[2]
		print(f"x: {x}, y:{y}, z:{z}")

		# assign the first two lengths of the robot arm
		# with shorter varible names
		l1 = self.l1
		l2 = self.l2

		# w reprents the radial length of the arm projected
		# onto the xy-plane
		w = np.sqrt((x**2 + y**2))

		# h represnts the the hypotenuse of the triangle
		# created between the height and the length of
		# the arm projection
		h = np.sqrt((w**2 + z**2))

		# if the distance to the point from the
		# base of the arm (h) is greater than the total
		# length of the arm then the goal is out of reach
		if h > (l1+l2):
			print("Unable to reach")
			bool = False
		elif h <= (l1+l2):
			print("Able to Reach")
			bool = True
		else:
			pass
		# bool represents the boolean of whether or
		# not the arm is able to reach its goal
		if bool:
			# Φ1, Φ2, and Φ4 reprent internal
			# angles to the triangle created by the two arm
			# segments
			# --- image of these angles can be found in
			# --- documentation as arm_triangle.png

			# Φ1 can be found by using the law of cosines
			if (l1**2+h**2-l2**2) >= (2*l1*h):
				phi1 = 0
			else:
				phi1 = np.arccos((l1**2+h**2-l2**2)/(2*l1*h))

			# Φ2 can also be found by using the law of cosines
			if phi1 == 0:
				phi2 = (np.pi)
			else:
				phi2 = np.arccos((l2**2+l1**2-h**2)/(2*l2*l1))

			# avoid divide by 0 error
			# Φ4 can be found by inverse tangent of the arm's
			# radial length and the height above the ground
			if w != 0:
				phi4 = np.arctan(abs(z)/abs(w))
			elif w == 0:
				phi4 = (np.pi/2)
			else:
				pass

			# angles 1, 2, 3, & 4 represent the angles relative
			# to the z axis, l1, l2, and the x axis respecively
			# --- image of these angles can be found in documentation
			# --- as arm_angles.png

			angle1 = (np.pi/2) - (phi4 + phi1)
			angle2 = np.pi - phi2
			angle3 = np.pi - angle1 - angle2

			# if x is equal to zero there will be a divide by zero error
			if x == 0:
				angle4 = 0
			elif x != 0:
				if ((x < 0) ^ (y < 0)):
					angle4 = np.arctan(y/x) + np.pi
				elif ((x < 0) & (y < 0)):
					angle4 = np.arctan(y/x) + np.pi
				else:
					angle4 = np.arctan(y/x)

			# path consists of a list of all of the angles at any given postion in degrees
			path = [np.degrees(angle1), np.degrees(
				angle2), np.degrees(angle3), np.degrees(angle4)]
		else:
			pass

		return path, bool
	
	def acceleration_path(self, inital, final, angle_adj):
		# "ani_path" consists of a list of the total time
		# taken to move each joint, the degree which
		# the joint will move through, and whether or
		# not the joint will reach maximum angular velocity
		# --- [total_time, angle_moved, reach_max_angle] ---
		step_config = [[0, 0, None], [0, 0, None], [0, 0, None], [0, 0, None], [0, 0, None]]
		a = self.accel
		ani_angle = [0, 0, 0, 0, 0]

		# for loop will create a list of all the times
		# and angles with the set max acceleration/
		# deceleration
		for w in range(0, len(step_config)):
			# adjust values if outside of range 0-360
			print(f"Initial Sample: {inital[w]}")
			if (inital[w] > 360):
				inital[w] = inital[w] - 360
			elif (inital[w] < 0):
				inital[w] = inital[w] + 360
			elif (final[w] > 360):
				final[w] = final[w] - 360
			elif (final[w] < 0):
				final[w] = final[w] + 360
			else:
				pass

			if final[w] != inital[w]:
				couple = [final[w], inital[w]]
				abs_couple = [abs(final[w]), abs(inital[w])]

				# determine shortest number of steps between angles
				if (abs(couple[0]-couple[1]) > 180):
					# if the absolute difference between the final and
					# inital angles is greater than 180 degrees
					# the end effector should travel in the opposite
					# direction to take less time
					negative_max = abs(max(couple) - 360)
					angle = negative_max + min(abs_couple)
					ani_angle[w] = negative_max + min(abs_couple)
					angle = angle*angle_adj[w]
					steps = int(angle)

					# check if max looks at whether or not the
					# joint will reach maximum angular velocity
					# given the distance to travel and the maximum
					# angular acceleration
					maximum = self.check_if_max(angle, a)
					# if the number of steps is zero
					# time will return 1 and angle will return
					# 0 to avoid a divide by zero error
					if steps == 0:
						step_config[w][0] = 1
						step_config[w][1] = 0
						step_config[w][2] = False

					# total_time finds the time to travel the
					# angular displacement at the max acceleration
					else:
						total_time, divs = self.find_total_time(angle, maximum, a)
						step_config[w][0] = total_time
						step_config[w][1] = angle
						step_config[w][2] = maximum

				elif (abs(couple[0]-couple[1]) <= 180):
					# if the absolute difference between the final
					# and inital angles is less than 180 the travel
					# path will be as short as possible
					angle = max(abs_couple) - min(abs_couple)
					ani_angle[w] = max(abs_couple) - min(abs_couple)
					angle = angle*angle_adj[w]
					steps = int(angle)

					# checks if max angular velocity is reached
					maximum = self.check_if_max(angle, a)

					# returns 1 and 0 to avoid 0/0 error
					if steps == 0:
						step_config[w][0] = 1
						step_config[w][1] = 0
						step_config[w][2] = False
					# finds total time based on shortest angular displacment
					else:
						total_time, divs = self.find_total_time(angle, maximum, a)
						step_config[w][0] = total_time
						step_config[w][1] = angle
						step_config[w][2] = maximum
				else:
					pass

			# returns 1 and 0 to avoid 0/0 error
			elif (final[w] == inital[w]):
				step_config[w][0] = 1
				step_config[w][1] = 0
				step_config[w][2] = False

			else:
				pass

		# copy is made so stepper motor order can be preserved
		copy = list(step_config)
		# print("Acceleation Path:", copy)

		# sorts based on time, search crit specifies that
		# the 0 index is the time varible
		step_config.sort(key=self.search_crit, reverse=True)

		# base_time is representative of half of the total
		# max travel time because half of the time is
		# equal to the time needed to accelerate and
		# decelerate
		base_time = step_config[0][0]/2
		t_time = step_config[0][0]
		print('Total Time', t_time)
		accel = [0, 0, 0, 0, 0]
		ani_accel = [0, 0, 0, 0, 0]
		for x in range(0, len(step_config)):
			if not copy[w][2]:
				a = np.radians(copy[x][1])/(base_time**2)
				accel[x] = a
				ani_accel[x] = np.radians(ani_angle[x])/(base_time**2)
			elif copy[w][2]:
				accel[x] = self.accel
				ani_accel[x] = self.accel
			else:
				pass
		print("accel: ", accel)
		print("ani_accel:", ani_accel)

		return accel, ani_accel, t_time

	def animation_path(self, inital, final, accel, ani_accel, t_time, angle_adj):
		# path will consist of a list of lists where
		# each index of the path represents the list of
		# angular values each joint will be at during the motion
		ani_path = [[], [], [], [], []]

		# define stepper variables
		self.step_angle = [0,0,0,0,0]
		self.step_path = [[],[],[],[],[]]
		self.step_key = []
		# # p is the number of divisions each animation path will
		# # broken up into
		p = 30
		self.p = p

		# division of time definition
		divisions = [0, 0, 0, 0, 0]

		# adjustment value for steps to angle
		R = self.micro_steps/(2*np.pi)
		for w in range(0, 5):
			a_a = ani_accel[w]
			a_s = accel[w]
			if final[w] != inital[w]:
				couple = [final[w], inital[w]]
				abs_couple = [abs(final[w]), abs(inital[w])]

				# adjust values if outside of range 0-360
				if (inital[w]) > 360:
					inital[w] = inital[w] - 360
				elif (inital[w]) < 0:
					inital[w] = inital[w] + 360
				elif (final[w]) > 360:
					final[w] = final[w] - 360
				elif (final[w]) < 0:
					final[w] = final[w] + 360
				else:
					pass

				# determine shortest number of steps between angles
				#
				# if the absolute difference between the final and
				# inital angles is greater than 180 degrees
				# the end effector should travel in the opposite
				# direction to take less time
				if (abs(couple[0] - couple[1]) > 180):
					# finds the distance to the goal in the opposing
					# direction which will be less than 180
					negative_max = abs(max(couple) - 360)

					# the displacement to the postion specified
					# is the sum of the absolute max + min
					angle = negative_max + min(abs_couple)
					self.step_angle[w] = angle*angle_adj[w]
					# steps is found by multipling the constant R
					# by the angle displaced
					steps = int(np.radians(self.step_angle[w])*R)

					# checks if maximum angular velocity is reached
					maximum = self.check_if_max(angle, a_a)

					# if there are 0 steps then the animation
					# ani_path will act as if there are 0
					if steps == 0:
						for x in range(1, self.p+1):
							ani_path[w].append(final[w])
					else:
						# finds total time
						total_time, divs = self.find_total_time(angle, maximum, a_a)

						# define division of times
						divisions[w] = divs
						# if the intial angle is the max angle then the end
						# the shortest distance to the goal point is past the
						# inital point
						if inital[w] == max(couple):
							# print("Step A")

							# if the angles are increasing the joint is accelerating towards the ground and is negative
							self.accel[w] = -self.accel[w]
							self.ani_accel[w] = -self.ani_accel[w]

							# iterate through the range of points defined by p
							for x in range(1, self.p+1):
								# t represents different increments of time
								t = (x*(total_time/self.p))

								# print("iterated time:", t)
								# print("trapezod function val:", np.degrees(self.trapezoid_function(t,angle,a,max_omega=maximum)))

								# the trapezoid function takes a the time,
								# anglular displacement and acceleration and
								# returns the value of the angle at the specified time
								ani_path[w].append(
									inital[w]+np.degrees(self.trapezoid_function(t, angle, a_a, max_omega=maximum)))
							self.step_key.append(True)
							for z in range(0, steps+1):
								count = z*(self.step_angle[w]/steps)
								self.step_path[w].append(self.inverted_trap(count, self.step_angle[w], a_s))
						# if the final angle is max then the shortest ani_path is
						# away from it
						elif final[w] == max(couple):
							# print("Step B")
							for x in range(1, self.p+1):
								# t represents different increments of time
								t = (x*(total_time/self.p))

								# print("iterated time:", t)
								# print("trapezod function val:", np.degrees(self.trapezoid_function(t,angle,a,max_omega=maximum)))

								# the trapezoid function takes a the time,
								# anglular displacement and acceleration and
								# returns the value of the angle at the specified time
								ani_path[w].append(
									inital[w]-np.degrees(self.trapezoid_function(t, angle, a_a, max_omega=maximum)))
							self.step_key.append(False)
							for z in range(0, steps+1):
								count = z*(self.step_angle[w]/steps)
								self.step_path[w].append(
									self.inverted_trap(count, self.step_angle[w], a_s))
								# define that the acceleration is in the negative direction

								# print("correct ani_path", w)
						else:
							print("error 1", w)

				# if the absoulte difference from the start and end
				# points is less than 180
				elif (abs(couple[0]-couple[1]) <= 180):
					# the angle needed to travel is the difference between
					# the absolute maximum and minimum
					angle = max(abs_couple) - min(abs_couple)

					# the number of steps is defined
					self.step_angle[w] = angle*angle_adj[w]
					steps = int(np.radians(self.step_angle[w])*R)

					# print("# of Steps: ", steps)

					# checks if the maximum angular velocity is reached
					maximum = self.check_if_max(angle, a_a)

					# print("angle: ", angle)
					# print("maximum: ", maximum)

					if steps == 0:
						for x in range(1, self.p+1):
							ani_path[w].append(final[w])
					else:
						# total time is found
						total_time, divs = self.find_total_time(angle, maximum, a_a)

						# define division of times
						divisions[w] = divs

						# print("Angle",w,"Total Time: ", total_time)

						# if the initial value is max then traveling towards
						# it is the fastest ani_path
						if inital[w] == max(couple):

							# print("Step C")

							# iterate through time to find ani_path
							for x in range(1, self.p+1):
								# time incrimentally found
								t = (x*(total_time/self.p))
								# print("iterated time:", t)

								# print("trapezod function val:", np.degrees(self.trapezoid_function(t,angle,a,max_omega=maximum)))

								# trapezoid function is used to find angle
								ani_path[w].append(
									inital[w]-np.degrees(self.trapezoid_function(t, angle, a_a, max_omega=maximum)))
							self.step_key.append(False)
							for z in range(0, steps+1):
								count = z*(self.step_angle[w]/steps)
								self.step_path[w].append(self.inverted_trap(count, self.step_angle[w], a_s))
								# print("correct ani_path", w)

						# if the final angle is greater than the inital
						# then the shortest ani_path is away from the initial
						elif final[w] == max(couple):
							# print("Step D")

							for x in range(1, self.p+1):
								# time incrimentally found
								t = (x*(total_time/self.p))

								# print("iterated time:", t)
								# print("trapezod function val:",np.degrees(self.trapezoid_function(t,angle,a,max_omega=maximum)))

								# trapezoid function is used to find angle
								ani_path[w].append(
									inital[w]+np.degrees(self.trapezoid_function(t, angle, a_a, max_omega=maximum)))

								# if the angles are increasing the joint is accelerating towards the ground and is negative
								self.accel[w] = -self.accel[w]
								self.ani_accel[w] = -self.ani_accel[w]
							self.step_key.append(True)
							for z in range(0, steps+1):
								count = z*(self.step_angle[w]/steps)
								self.step_path[w].append(self.inverted_trap(count, self.step_angle[w], a_s))
			# if the inital and final angles are the same
			# then no distance is travelled
			elif (final[w] == inital[w]):
				for x in range(1, self.p+1):
					ani_path[w].append(final[w])
				divisions[w] = [t_time, 0, 0]
				# print("appended 0")

			else:
				pass

		return ani_path, divisions

	def find_total_time(self, angle, max, accel):
		angle = np.radians(angle)
		decel = accel
		divs = [0, 0, 0]
		if not max:
			# found by adding the angular displacement of the
			# acceleration to the displacement of the deceleration
			# and solving for time_a using the relationship that
			# the absolute value of accel*time_a = decel*time_c
			time_a = np.sqrt((2*angle)/(accel+(accel**2/decel)))

			# found by using the relationship that
			# the absolute value of accel*time_a = decel*time_c
			time_c = time_a*(accel/decel)

			# define divs
			divs[0] = time_a
			divs[2] = time_c

			# sum components
			total_time = time_a + time_c

		elif max:
			# found by using the relationship that wt = a
			time_a = self.w_max/accel

			# found by dividing the displacement at max angular
			# velocity by the max velocity
			time_b = (angle - ((self.w_max**2/(2*accel)) +
					(self.w_max**2/(2*decel))))/self.w_max

			# same as time_a
			time_c = self.w_max/decel

			# define divs
			divs[0] = time_a
			divs[1] = time_b
			divs[2] = time_c

			# sum components
			total_time = time_a + time_b + time_c

		return total_time, divs

	def check_if_max(self, angle, accel):
		angle = np.radians(angle)
		decel = accel
		# max_theta represents the angular displacement of a
		# stepper motor accelerating and decelerating at the maximum
		# acceleration value.
		#
		# So if the angle needed to travel is greater than the max
		# angular displacement, then the maximum angular velocity will be
		# reached and max will return true.
		max_theta = (self.w_max**2/(2*accel)) + (self.w_max**2/(2*decel))
		max = None
		if angle < max_theta:
			max = False
		elif angle >= max_theta:
			max = True

		return max

	def search_crit(self, elem):
		# specifies that the 0 index is the
		# time variable
		return elem[0]

	def trapezoid_function(self, time, theta, accel, max_omega=False):
		# define total anglular displacement as theta
		theta = np.radians(theta)
		decel = accel

		# if the max angular velocity isn't reached
		if not max_omega:
			# time a is time inital acceleration takes
			time_a = np.sqrt((2*theta)/(accel+(accel**2/decel)))

			# before time a is reached the angular diplacement is
			# due directly to acceleration
			if time <= time_a:
				r = 0.5*accel*time**2

			# after time to finish acceleration
			# displacement is now due to inital displacement
			# plus angular velocity plus deceleration
			elif time > time_a:
				adj_time = time - time_a
				r = (0.5*accel*time_a**2) + (accel*time_a *
                                 adj_time) - (0.5*decel*adj_time**2)
			else:
				pass
		# if the maximum angular velocity is reached
		elif max_omega:
			# time a is the time to reach max
			# angular acceleration
			time_a = self.w_max/accel

			# time b is the time to get the extra angular
			# displacement not gotten from acceleration and
			# deceleration
			time_b = (theta - (self.w_max**2/(2*accel)) +
			          (self.w_max**2/(2*decel)))/self.w_max

			# before time a is reached the angular diplacement is
			# due directly to acceleration
			if time < time_a:
				r = 0.5*accel*time**2

			# after intial acceleration time but before
			# deceleration displacement is due to initial
			# displacement plus max angular velocity displacement
			elif (time_a <= time < (time_b + time_a)):
				r = (0.5*accel*time_a**2) + accel*time_a*(time-time_a)

			# after time b, displacement is due to current displacemnt
			# plus displacement due to angular velocity
			elif (time >= (time_b + time_a)):
				r = (0.5*accel*time_a**2) + (accel*time_a*time_b) - (0.5*decel*(time-time_a-time_b)**2)

			else:
				pass
		else:
			pass

		return r

	def inverted_trap(self, count, total, accel):
		decel = accel
		count = np.radians(count)
		total = np.radians(total)
		time_half = np.sqrt((2*total)/(accel+(accel**2/decel)))
		t_try = np.sqrt((2*count)/accel)
		if t_try < time_half:
			t_ret = t_try
		elif t_try >= time_half:
			a = -0.5*accel
			b = accel*time_half
			c = count - (total/2)
			if ((b**2)+(4*a*c)) < 0:
				root = np.sqrt((b**2)-(4*a*c))
			elif ((b**2)+(4*a*c)) >= 0:
				root = np.sqrt((b**2)+(4*a*c))

			t_ret = -((b-root)/(2*a)) + time_half
		else:
			print("Error -- Inverted Trap -- path.py")
		
		return t_ret
	
	def stepper_path(self, times):
		short_delay_time = 0.005
		print(f"Stepper Delay Times: {times[0]}")
		for w in range(0, len(times)):
			for i in range(1, len(times[w])):
				times[w][i] = times[w][i] + short_delay_time

		short_delay = [[short_delay_time], [short_delay_time], [short_delay_time], 
						[short_delay_time], [short_delay_time]]

		for w in range(0, len(times)):
			for i in range(0, len(times[w])):
				short_delay[w].append(times[w][i] + short_delay_time)

		return times, short_delay

