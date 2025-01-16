from MCEVS.Missions.Segments.HoverClimb.Constant_Speed import HoverClimbConstantSpeed
from MCEVS.Missions.Segments.HoverClimb.Constant_Acceleration import HoverClimbConstantAcceleration
from MCEVS.Missions.Segments.Climb.Constant_Vy_Constant_Vx import ClimbConstantVyConstantVx
from MCEVS.Missions.Segments.Climb.Constant_Vy_Constant_Ax import ClimbConstantVyConstantAx
from MCEVS.Missions.Segments.Transition.Constant_Acceleration import TransitionConstantAcceleration
from MCEVS.Missions.Segments.Cruise.Constant_Speed import CruiseConstantSpeed
from MCEVS.Missions.Segments.Descent.Constant_Vy_Constant_Vx import DescentConstantVyConstantVx
from MCEVS.Missions.Segments.Descent.Constant_Vy_Constant_Ax import DescentConstantVyConstantAx
from MCEVS.Missions.Segments.Descent.No_Credit import NoCreditDescent
from MCEVS.Missions.Segments.HoverDescent.Constant_Deceleration import HoverDescentConstantDeceleration
from MCEVS.Missions.Segments.HoverDescent.Constant_Speed import HoverDescentConstantSpeed
from MCEVS.Missions.Segments.Hover.Stay import HoverStay
from MCEVS.Missions.Segments.Others.Constant_Power import ConstantPower
from MCEVS.Missions.Segments.Others.Reserve_Cruise import ReserveCruise
import numpy as np
import matplotlib.pyplot as plt

class Mission(object):
	"""
	A container for a mission. Users can add segments to this container.
	"""
	def __init__(self, takeoff_altitude:float, n_repetition:float):
		super(Mission, self).__init__()
		self.takeoff_altitude = takeoff_altitude # ASL (Above Sea Level)
		self.curr_id = 0
		self.segments = []
		self.n_segments = 0 # number of segments excluding reserve segment
		self.x 	= [np.array([0.0])]
		self.y 	= [np.array([0.0])]
		self.vx = [np.array([0.0])]
		self.vy = [np.array([0.0])]
		self.ax = [np.array([0.0])]
		self.ay = [np.array([0.0])]
		self.t 	= [np.array([0.0])]

		# how many times the mission should be repeated
		# (not including reserve mission)
		self.n_repetition = n_repetition
		self.reserve_mission_duration = 0.0

		# Performance
		self.P 	= {}	# power
		self.DL = {}	# disk loading

	def add_segment(self, name:str, kind:str, n_discrete=10, **kwargs):
		"""
		Adding a segment sequentially
		name : name of the segment up to the user
		kind : kind of the segment;
			   available:
			   		'HoverClimbConstantSpeed',
			   		'HoverClimbConstatAcceleration',
			   		'ClimbConstantVyConstantVx',
			   		'ClimbConstantVyConstantAx',
			   		'CruiseConstantSpeed',
			   		'DescentConstantVyConstantVx',
			   		'DescentConstantVyConstantAx',
			   		'HoverDescentConstantDeceleration',
			   		'HoverDescentConstantSpeed',
			   		'HoverStay'
			   		'ConstantPower'
			   		'ReserveCruise'
		"""
		self.curr_id += 1

		if kind == 'ConstantPower':
			self.segments.append(ConstantPower(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverClimbConstantSpeed':
			self.segments.append(HoverClimbConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverClimbConstantAcceleration':
			self.segments.append(HoverClimbConstantAcceleration(id=self.curr_id, name=name, initial_speed=self.vy[-1][-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'ClimbConstantVyConstantVx':
			self.segments.append(ClimbConstantVyConstantVx(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'ClimbConstantVyConstantAx':
			self.segments.append(ClimbConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1][-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'TransitionConstantAcceleration':
			self.segments.append(TransitionConstantAcceleration(id=self.curr_id, name=name, initial_speed=self.vx[-1][-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'CruiseConstantSpeed':
			self.segments.append(CruiseConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'DescentConstantVyConstantVx':
			self.segments.append(DescentConstantVyConstantVx(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'DescentConstantVyConstantAx':
			self.segments.append(DescentConstantVyConstantAx(id=self.curr_id, name=name, initial_speed_X=self.vx[-1][-1], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'NoCreditDescent':
			self.segments.append(NoCreditDescent(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverDescentConstantDeceleration':
			self.segments.append(HoverDescentConstantDeceleration(id=self.curr_id, name=name, initial_speed=kwargs['initial_speed'], n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverDescentConstantSpeed':
			self.segments.append(HoverDescentConstantSpeed(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'HoverStay':
			self.segments.append(HoverStay(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		elif kind == 'ReserveCruise':
			self.segments.append(ReserveCruise(id=self.curr_id, name=name, n_discrete=n_discrete, kwargs=kwargs))

		self.segments[self.curr_id-1]._initialize()

		# Calculate the next timestamp, position, velocity, and acceleration
		t_next = self.segments[self.curr_id-1]._calc_time(self.t)
		self.x, self.y = self.segments[self.curr_id-1]._calc_position(self.x, self.y, t_next)
		self.vx, self.vy = self.segments[self.curr_id-1]._calc_velocity(self.vx, self.vy, t_next)
		self.ax, self.ay = self.segments[self.curr_id-1]._calc_acceleration(self.ax, self.ay, t_next)

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

		# Counting number of segments
		if kind == 'ReserveCruise':
			self.reserve_mission_duration += self.segments[self.curr_id-1].duration
		else:
			self.n_segments +=1

	def print_info(self):
		print('Mission Info')
		print(f'\tNumber of segment(s) = {len(self.segments)}')
		print('\tList of segment(s) :')
		for segment in self.segments:
			print(segment._info())


