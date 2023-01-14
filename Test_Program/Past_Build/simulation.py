import tkinter as tk
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

class Simulation(object):
	def __init__(self):
		pass
		# define class variables for simulation
		# self.fig = plt.figure()
		# self.ax = Axes3D(self.fig)
		# self.ani_running = False

	def init_sim_ax(self):
		max_lim = 700
		self.ax.set_xlim3d([-max_lim, max_lim])
		self.ax.set_xlabel('X')
		#
		self.ax.set_ylim3d([-max_lim, max_lim])
		self.ax.set_ylabel('Y')
		#
		self.ax.set_zlim3d([0.0, max_lim])
		self.ax.set_zlabel('Z')
		#
		self.ax.set_title('Robot Arm Physics Simulation')

		# NOTE: Can't pass empty arrays into 3d version of plot()
		set = [[[], []], [[], []], [[], []], [[], []], [[], []]]
		self.lineh, = self.ax.plot(set[0][0], set[0][1], 'bo', linestyle='solid')
		self.line0, = self.ax.plot(set[0][0], set[0][1], 'bo', linestyle='solid')
		self.line1, = self.ax.plot(set[1][0], set[1][1], 'bo', linestyle='solid')
		self.line2, = self.ax.plot(set[2][0], set[2][1], 'bo', linestyle='solid')
		self.line3, = self.ax.plot(set[3][0], set[3][1], 'bo', linestyle='solid')

	def init_sim_frame(self, sim_frame):
		# self.fig, self.ax = plt.subplots(1,1)
		self.sim_canvas = FigureCanvasTkAgg(self.fig, master=sim_frame)
		self.sim_canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
		self.sim_canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
		self.sim_canvas.mpl_connect('button_press_event', self.ax._button_press)
		self.sim_canvas.mpl_connect('button_release_event', self.ax._button_release)
		self.sim_canvas.mpl_connect('motion_notify_event', self.ax._on_move)

	def ani_func(self, num, path, lineh, line0, line1, line2, line3):
		lines = path[num]

		scale = 1000
		self.lineh.set_data_3d(np.dot(lines[0][0],scale), np.dot(lines[0][1],scale), np.dot(lines[0][2],scale))
		#
		self.line0.set_data_3d(np.dot(lines[1][0],scale), np.dot(lines[1][1],scale), np.dot(lines[1][2],scale))
		#
		self.line1.set_data_3d(np.dot(lines[2][0],scale), np.dot(lines[2][1],scale), np.dot(lines[2][2],scale))
		#
		self.line2.set_data_3d(np.dot(lines[3][0],scale), np.dot(lines[3][1],scale), np.dot(lines[3][2],scale))
		#
		self.line3.set_data_3d(np.dot(lines[4][0],scale),np.dot(lines[4][1],scale),np.dot(lines[4][2],scale))
		#
		self.sim_canvas.draw()
		self.sim_canvas.flush_events()

		return self.lineh, self.line0, self.line1, self.line2, self.line3

	def call_animation(self, static_pkg, suite):
		if not self.ani_running:
			print("Setting Up Simulation")
			self.init_sim_ax()
			scale = 1000
			self.goals = self.ax.scatter(np.dot(static_pkg['goals'][0],scale),np.dot(static_pkg['goals'][1],scale),
											np.dot(static_pkg['goals'][2],scale), color = 'r')
			self.outline = self.ax.scatter(np.dot(static_pkg['outline'][0],scale),np.dot(static_pkg['outline'][1],scale),
											np.dot(static_pkg['outline'][2],scale), color = 'g')
			numDataPoints = len(suite)
			self.line_ani = animation.FuncAnimation(self.fig, self.ani_func, frames=numDataPoints,
						fargs=(suite,self.lineh, self.line0, self.line1, self.line2, self.line3), interval=50, blit=False)
			self.sim_canvas.draw()
			self.ani_running = True
		else:
			print("Cannot Make New Animation Becuase One Is Already Running!")

	def stop_animation(self):
		if self.ani_running:
			self.line_ani.event_source.stop()
			self.ax.clear()
			self.ani_running = False
