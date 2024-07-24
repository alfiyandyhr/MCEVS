from MCEVS.Missions.Segments.HoverClimb.Constant_Speed import HoverClimbConstantSpeed
from MCEVS.Missions.Segments.HoverClimb.Constant_Acceleration import HoverClimbConstantAcceleration
from MCEVS.Missions.Segments.Climb.Constant_Vy_Constant_Ax import ClimbConstantVyConstantAx
from MCEVS.Missions.Segments.Cruise.Constant_Speed import CruiseConstantSpeed
from MCEVS.Missions.Segments.Descent.Constant_Vy_Constant_Ax import DescentConstantVyConstantAx
from MCEVS.Missions.Segments.HoverDescent.Constant_Deceleration import HoverDescentConstantDeceleration
from MCEVS.Missions.Segments.HoverDescent.Constant_Speed import HoverDescentConstantSpeed
from MCEVS.Missions.Segments.Hover.Stay import HoverStay
import numpy as np
import matplotlib.pyplot as plt

class Mission(object):
	"""
	A container for a mission. Users can add segments to this container.
	"""
	def __init__(self):
		super(Mission, self).__init__()
		self.curr_id = 0
		self.segments = []
		self.x = np.array([0.0])
		self.y = np.array([0.0])
		self.vx = np.array([0.0])
		self.vy = np.array([0.0])
		self.ax = np.array([0.0])
		self.ay = np.array([0.0])
		self.t = np.array([0.0])

	def add_segment(self, name:str, kind:str, n_discrete=10, **kwargs):
		"""
		Adding a segment sequentially
		name : name of the segment up to the user
		kind : kind of the segment;
			   available:
			   		"HoverClimbConstantSpeed",
			   		"TransitionClimb",
			   		"CruiseConstantSpeed",
			   		"TransitionDescent",
			   		"HoverDescent",
			   		"HoverStay"
		"""
		self.curr_id += 1

		if kind == 'HoverClimbConstantSpeed':
			self.segments.append(HoverClimbConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == "HoverClimbConstantAcceleration":
			self.segments.append(HoverClimbConstantAcceleration(id=self.curr_id, name=name, initial_speed=self.vy[-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == "ClimbConstantVyConstantAx":
			self.segments.append(ClimbConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'CruiseConstantSpeed':
			self.segments.append(CruiseConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == "DescentConstantVyConstantAx":
			self.segments.append(DescentConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverDescentConstantDeceleration':
			self.segments.append(HoverDescentConstantDeceleration(id=self.curr_id, name=name, initial_speed=kwargs['initial_speed'], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverDescentConstantSpeed':
			self.segments.append(HoverDescentConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverStay':
			self.segments.append(HoverStay(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		self.segments[self.curr_id-1]._initialize()

		# Calculate the next timestamp, position, velocity, and acceleration
		t_next = self.segments[self.curr_id-1]._calc_time(self.t)
		self.x, self.y = self.segments[self.curr_id-1]._calc_position(self.x, self.y, t_next)
		self.vx, self.vy = self.segments[self.curr_id-1]._calc_velocity(self.vx, self.vy, t_next)
		self.ax, self.ay = self.segments[self.curr_id-1]._calc_acceleration(self.ax, self.ay, t_next)

		# Update the time list
		self.t = np.concatenate((self.t, t_next))

		# Delete first point
		if self.curr_id == 1:
			self.x = self.x[1:]
			self.y = self.y[1:]
			self.vx = self.vx[1:]
			self.vy = self.vy[1:]
			self.ax = self.ax[1:]
			self.ay = self.ay[1:]
			self.t = self.t[1:]

	def print(self):
		print('Mission Info')
		print(f'\tNumber of segment(s) = {len(self.segments)}')
		print('\tList of segment(s) :')
		for segment in self.segments:
			print(segment._info())

	def visualize(self):

		# Plot position
		plt.plot(self.x/1000.0, self.y, '-o')
		plt.xlabel('Distance ($km$) in X-dir')
		plt.ylabel('AGL Altitude ($m$) in Y-dir')
		plt.title('Aircraft position y vs x')
		plt.grid()
		plt.show()

		# Plot bulk
		fig, axs = plt.subplots(nrows=3, ncols=2, sharex=False)
		axs[0,0].plot(self.t/60.0, self.x/1000.0, '-o')
		axs[0,0].set_ylabel('Distance ($km$) in X-dir')
		axs[0,0].grid()
		axs[0,1].plot(self.t/60.0, self.y, '-o')
		axs[0,1].set_ylabel('AGL Altitude ($m$) in Y-dir')
		axs[0,1].grid()
		axs[1,0].plot(self.t/60.0, self.vx*3.6, '-o')
		axs[1,0].set_ylabel('Velocity x-dir ($km/h$)')
		axs[1,0].grid()
		axs[1,1].plot(self.t/60.0, self.vy, '-o')
		axs[1,1].set_ylabel('Velocity y-dir ($m/s$)')
		axs[1,1].grid()
		axs[2,0].plot(self.t/60.0, self.ax, '-o')
		axs[2,0].set_ylabel('Acceleration x-dir ($m/s^2$)')
		axs[2,0].set_xlabel('Time ($min$)')
		axs[2,0].grid()
		axs[2,1].plot(self.t/60.0, self.ay, '-o')
		axs[2,1].set_ylabel('Acceleration y-dir ($m/s^2$)')
		axs[2,1].set_xlabel('Time ($min$)')
		axs[2,1].grid()
		plt.show()




