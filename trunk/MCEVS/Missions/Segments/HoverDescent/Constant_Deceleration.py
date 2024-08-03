import numpy as np

class HoverDescentConstantDeceleration():
	"""
	docstring for HoverDescentConstantDeceleration
	kwargs:
		final_speed		: final hover climb speed [m/s]
		deceleration 	: constant hover climb deceleration [m/s**2]
		distance 		: distance of the hover climb [m]
		duration		: duration of the hover climb [s]
	"""
	def __init__(self, id:int, name:str, initial_speed:float, kwargs:dict, n_discrete=10):
		super(HoverDescentConstantDeceleration, self).__init__()
		self.id = id 									# segment id
		self.name = name 								# segment name
		self.kind = 'HoverDescentConstantDeceleration'	# segment kind
		self.n_discrete = n_discrete					# mission discretization
		self.kwargs = kwargs

		self.AoA = 90.0 # deg
		self.initial_speed = initial_speed
		self.final_speed = None
		self.distance = None
		self.duration = None
		self.deceleration = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'final_speed':
				self.final_speed = float(self.kwargs[item])
			elif item == 'distance':
				self.distance = float(self.kwargs[item])
			elif item == 'duration':
				self.duration = float(self.kwargs[item])
			elif item == 'deceleration':
				self.deceleration = float(self.kwargs[item])
		
		try:
			if self.deceleration is None:
				if self.duration is None:
					self.deceleration = (self.initial_speed**2 - self.final_speed**2)/(2 * self.distance)
					self.duration = (self.initial_speed - self.final_speed)/self.deceleration
				if self.distance is None:
					self.deceleration = (self.initial_speed - self.final_speed)/self.duration
					self.distance = (self.initial_speed**2 - self.final_speed**2)/(2 * self.deceleration)
				if self.final_speed is None:
					self.deceleration = 2 * (self.initial_speed * self.duration - self.distance) / (self.duration**2)
					self.final_speed = self.initial_speed - self.deceleration * self.duration
			if self.final_speed is None:
				if self.distance is None:
					self.final_speed = self.initial_speed - self.deceleration * self.duration
					self.distance = self.initial_speed * self.duration - 0.5 * self.deceleration * self.duration**2
				if self.duration is None:
					self.final_speed = np.sqrt(self.initial_speed**2 - 2 * self.deceleration * self.distance)
					self.duration = (self.initial_speed - self.final_speed) / self.deceleration
			if self.distance is None:
				if self.duration is None:
					self.distance = (self.initial_speed**2 - self.final_speed**2)/(2 * self.deceleration)
					self.duration = (self.initial_speed - self.final_speed)/self.deceleration
		except:
			raise NameError("Need to define at least two of the followings: Final_Speed, Distance, Duration, Deceleration")

	def _calc_time(self, t_list):
		t0 = t_list[-1][-1]
		t_next = np.linspace(t0, t0+self.duration, self.n_discrete+1)
		return t_next

	def _calc_position(self, x_list, y_list, t_next):
		x0, y0, t0 = x_list[-1][-1], y_list[-1][-1], t_next[0]
		# Position under constant Deceleration
		x_next = x0 + np.zeros_like(t_next)
		y_next = y0 - self.initial_speed * (t_next - t0) + 0.5 * self.deceleration * (t_next - t0)**2
		x_list.append(x_next)
		y_list.append(y_next)
		return x_list, y_list

	def _calc_velocity(self, vx_list, vy_list, t_next):
		vx0, vy0, t0 = 0.0, -self.initial_speed, t_next[0]
		# Velocity under constant deceleration in y
		vx_next = vx0 + np.zeros_like(t_next)
		vy_next = vy0 - (-self.deceleration) * (t_next - t0)
		vx_list.append(vx_next)
		vy_list.append(vy_next)
		return vx_list, vy_list

	def _calc_acceleration(self, ax_list, ay_list, t_next):
		ax0, ay0, t0 = ax_list[-1], -self.deceleration, t_next[0]
		# Zero Deceleration in x, and constant Deceleration in y
		ax_next = np.zeros_like(t_next)
		ay_next = - self.deceleration * np.ones_like(t_next)
		ax_list.append(ax_next)
		ay_list.append(ay_next)
		return ax_list, ay_list

	def _info(self):
		info = f'\t\tSegment ID = {self.id}\n'
		info += f'\t\tSegment Name: {self.name}\n'
		info += f'\t\t\tDeceleration = {self.deceleration} m/s2\n'
		info += f'\t\t\tDistance = {self.distance} m\n'
		info += f'\t\t\tDuration = {self.duration} s\n'
		info += f'\t\t\tFinal Speed = {self.final_speed} m/s\n'
		return info

	def print_info(self):
		print(f'Segment ID = {self.id}')
		print(f'Segment Name: {self.name}')
		print(f'\tDeceleration = {self.deceleration} m/s2')
		print(f'\tDistance = {self.distance} m')
		print(f'\tDuration = {self.duration} s')
		print(f'\tFinal Speed = {self.final_speed} m/s')