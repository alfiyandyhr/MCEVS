from MCEVS.Missions.Segments.HoverClimb.Constant_Speed import HoverClimbConstantSpeed
from MCEVS.Missions.Segments.HoverClimb.Constant_Acceleration import HoverClimbConstantAcceleration
from MCEVS.Missions.Segments.Climb.Constant_Vy_Constant_Vx import ClimbConstantVyConstantVx
from MCEVS.Missions.Segments.Climb.Constant_Vy_Constant_Ax import ClimbConstantVyConstantAx
from MCEVS.Missions.Segments.Cruise.Constant_Speed import CruiseConstantSpeed
from MCEVS.Missions.Segments.Descent.Constant_Vy_Constant_Vx import DescentConstantVyConstantVx
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
		self.segments = {}
		self.x = [np.array([0.0])]
		self.y = [np.array([0.0])]
		self.vx = [np.array([0.0])]
		self.vy = [np.array([0.0])]
		self.ax = [np.array([0.0])]
		self.ay = [np.array([0.0])]
		self.t = [np.array([0.0])]

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
			self.segments[self.curr_id] = [kind, HoverClimbConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == "HoverClimbConstantAcceleration":
			self.segments[self.curr_id] = [kind, HoverClimbConstantAcceleration(id=self.curr_id, name=name, initial_speed=self.vy[-1][-1], n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == "ClimbConstantVyConstantVx":
			self.segments[self.curr_id] = [kind, ClimbConstantVyConstantVx(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == "ClimbConstantVyConstantAx":
			self.segments[self.curr_id] = [kind, ClimbConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1][-1], n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == 'CruiseConstantSpeed':
			self.segments[self.curr_id] = [kind, CruiseConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == "DescentConstantVyConstantVx":
			self.segments[self.curr_id] = [kind, DescentConstantVyConstantVx(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == "DescentConstantVyConstantAx":
			self.segments[self.curr_id] = [kind, DescentConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1][-1], n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == 'HoverDescentConstantDeceleration':
			self.segments[self.curr_id] = [kind, HoverDescentConstantDeceleration(id=self.curr_id, name=name, initial_speed=kwargs['initial_speed'], n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == 'HoverDescentConstantSpeed':
			self.segments[self.curr_id] = [kind, HoverDescentConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		elif kind == 'HoverStay':
			self.segments[self.curr_id] = [kind, HoverStay(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs)]

		self.segments[self.curr_id][1]._initialize()

		# Calculate the next timestamp, position, velocity, and acceleration
		t_next = self.segments[self.curr_id][1]._calc_time(self.t)
		self.x, self.y = self.segments[self.curr_id][1]._calc_position(self.x, self.y, t_next)
		self.vx, self.vy = self.segments[self.curr_id][1]._calc_velocity(self.vx, self.vy, t_next)
		self.ax, self.ay = self.segments[self.curr_id][1]._calc_acceleration(self.ax, self.ay, t_next)

		# Append the time list
		self.t.append(t_next)

		# Delete first point
		if self.curr_id == 1:
			self.x = self.x[1:]
			self.y = self.y[1:]
			self.vx = self.vx[1:]
			self.vy = self.vy[1:]
			self.ax = self.ax[1:]
			self.ay = self.ay[1:]
			self.t = self.t[1:]

	def print_info(self):
		print('Mission Info')
		print(f'\tNumber of segment(s) = {len(self.segments)}')
		print('\tList of segment(s) :')
		for id in range(1, len(self.segments)+1):
			print(self.segments[id][1]._info())


