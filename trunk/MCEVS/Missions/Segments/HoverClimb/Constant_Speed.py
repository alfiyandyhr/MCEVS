import numpy as np

class HoverClimbConstantSpeed():
	"""
	docstring for HoverClimbConstantSpeed
	kwargs:
		speed 		: constant hover climb speed [m/s]
		distance 	: distance of the hover climb [m]
		duration	: duration of the hover climb [s]
	"""
	def __init__(self, id:int, name:str, kwargs:dict, n_discrete=10):
		super(HoverClimbConstantSpeed, self).__init__()
		self.id = id 							# segment id
		self.name = name 						# segment name
		self.kind = 'HoverClimbConstantSpeed'	# segment kind
		self.n_discrete = n_discrete			# mission discretization
		self.kwargs = kwargs

		self.AoA = -90.0 # deg
		self.speed = None
		self.distance = None
		self.duration = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'speed':
				self.speed = float(self.kwargs[item])
			elif item == 'distance':
				self.distance = float(self.kwargs[item])
			elif item == 'duration':
				self.duration = float(self.kwargs[item])
		
		try:
			if self.speed is None:
				self.speed = self.distance / self.duration
			if self.distance is None:
				self.distance = self.speed * self.duration
			if self.duration is None:
				self.duration = self.distance / self.speed
		except:
			raise NameError("Need to define at least two of the followings: Speed, Distance, Duration")

	def _calc_time(self, t_list):
		t0 = t_list[-1][-1]
		t_next = np.linspace(t0, t0+self.duration, self.n_discrete+1)
		return t_next

	def _calc_position(self, x_list, y_list, t_next):
		x0, y0, t0 = x_list[-1][-1], y_list[-1][-1], t_next[0]
		# Position under constant speed
		x_next = x0 + 0.0 * (t_next - t0)
		y_next = y0 + self.speed * (t_next - t0)
		x_list.append(x_next)
		y_list.append(y_next)
		return x_list, y_list

	def _calc_velocity(self, vx_list, vy_list, t_next):
		vx0, vy0, t0 = vx_list[-1][-1], self.speed, t_next[0]
		# Velocity under zero acceleration
		vx_next = vx0 + 0.0 * (t_next - t0)
		vy_next = vy0 + 0.0 * (t_next - t0)
		vx_list.append(vx_next)
		vy_list.append(vy_next)
		return vx_list, vy_list

	def _calc_acceleration(self, ax_list, ay_list, t_next):
		# Zero acceleration
		ax_next = np.zeros_like(t_next)
		ay_next = np.zeros_like(t_next)
		ax_list.append(ax_next)
		ay_list.append(ay_next)
		return ax_list, ay_list

	def _info(self):
		info = f'\t\tSegment ID = {self.id}\n'
		info += f'\t\tSegment Name: {self.name}\n'
		info += f'\t\t\tSpeed = {self.speed} m/s\n'
		info += f'\t\t\tDistance = {self.distance} m\n'
		info += f'\t\t\tDuration = {self.duration} s'
		return info

	def print_info(self):
		print(f'Segment ID = {self.id}')
		print(f'Segment Name: {self.name}')
		print(f'\tSpeed = {self.speed} m/s')
		print(f'\tDistance = {self.distance} m')
		print(f'\tDuration = {self.duration} s')

