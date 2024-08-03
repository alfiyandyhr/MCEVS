import numpy as np

class HoverClimbConstantAcceleration():
	"""
	docstring for HoverClimbConstantAcceleration
	kwargs:
		final_speed		: final hover climb speed [m/s]
		acceleration 	: constant hover climb acceleration [m/s**2]
		distance 		: distance of the hover climb [m]
		duration		: duration of the hover climb [s]
	"""
	def __init__(self, id:int, name:str, initial_speed:float, kwargs:dict, n_discrete=10):
		super(HoverClimbConstantAcceleration, self).__init__()
		self.id = id 									# segment id
		self.name = name 								# segment name
		self.kind = 'HoverClimbConstantAcceleration'	# segment kind
		self.n_discrete = n_discrete					# mission discretization
		self.kwargs = kwargs
		
		self.AoA = -90.0 # deg
		self.initial_speed = initial_speed
		self.final_speed = None
		self.acceleration = None
		self.distance = None
		self.duration = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'final_speed':
				self.final_speed = float(self.kwargs[item])
			elif item == 'distance':
				self.distance = float(self.kwargs[item])
			elif item == 'duration':
				self.duration = float(self.kwargs[item])
			elif item == 'acceleration':
				self.acceleration = float(self.kwargs[item])
		
		try:
			if self.acceleration is None:
				if self.duration is None:
					self.acceleration = (self.final_speed**2 - self.initial_speed**2)/(2 * self.distance)
					self.duration = (self.final_speed - self.initial_speed)/self.acceleration
				if self.distance is None:
					self.acceleration = (self.final_speed - self.initial_speed)/self.duration
					self.distance = (self.final_speed**2 - self.initial_speed**2)/(2 * self.acceleration)
				if self.final_speed is None:
					self.acceleration = 2 * (self.distance - self.initial_speed * self.duration) / (self.duration**2)
					self.final_speed = self.initial_speed + self.acceleration * self.duration
			if self.final_speed is None:
				if self.distance is None:
					self.final_speed = self.initial_speed + self.acceleration * self.duration
					self.distance = self.initial_speed * self.duration + 0.5 * self.acceleration * self.duration**2
				if self.duration is None:
					self.final_speed = np.sqrt(self.initial_speed**2 + 2 * self.acceleration * self.distance)
					self.duration = (self.final_speed - self.initial_speed) / self.acceleration
			if self.distance is None:
				if self.duration is None:
					self.distance = (self.final_speed**2 - self.initial_speed**2)/(2 * self.acceleration)
					self.duration = (self.final_speed - self.initial_speed)/self.acceleration
		except:
			raise NameError("Need to define at least two of the followings: Final_Speed, Distance, Duration, Acceleration")

	def _calc_time(self, t_list):
		t0 = t_list[-1][-1]
		t_next = np.linspace(t0, t0+self.duration, self.n_discrete+1)
		return t_next

	def _calc_position(self, x_list, y_list, t_next):
		x0, y0, t0 = x_list[-1][-1], y_list[-1][-1], t_next[0]
		# Position under constant acceleration
		x_next = x0 + np.zeros_like(t_next)
		y_next = y0 + self.initial_speed * (t_next - t0) + 0.5 * self.acceleration * (t_next - t0)**2
		x_list.append(x_next)
		y_list.append(y_next)
		return x_list, y_list

	def _calc_velocity(self, vx_list, vy_list, t_next):
		vx0, vy0, t0 = vx_list[-1][-1], vy_list[-1][-1], t_next[0]
		# Velocity under constant acceleration in y
		vx_next = vx0 + np.zeros_like(t_next)
		vy_next = vy0 + self.acceleration * (t_next - t0)
		vx_list.append(vx_next)
		vy_list.append(vy_next)
		return vx_list, vy_list

	def _calc_acceleration(self, ax_list, ay_list, t_next):
		# Zero acceleration in x, and constant acceleration in y
		ax_next = np.zeros_like(t_next)
		ay_next = self.acceleration * np.ones_like(t_next)
		ax_list.append(ax_next)
		ay_list.append(ay_next)
		return ax_list, ay_list

	def _info(self):
		info = f'\t\tSegment ID = {self.id}\n'
		info += f'\t\tSegment Name: {self.name}\n'
		info += f'\t\t\tAcceleration = {self.acceleration} m/s2\n'
		info += f'\t\t\tDistance = {self.distance} m\n'
		info += f'\t\t\tDuration = {self.duration} s\n'
		info += f'\t\t\tFinal Speed = {self.final_speed} m/s'
		return info

	def print_info(self):
		print(f'Segment ID = {self.id}')
		print(f'Segment Name: {self.name}')
		print(f'\tAcceleration = {self.acceleration} m/s2')
		print(f'\tDistance = {self.distance} m')
		print(f'\tDuration = {self.duration} s')
		print(f'\tFinal Speed = {self.final_speed} m/s')

