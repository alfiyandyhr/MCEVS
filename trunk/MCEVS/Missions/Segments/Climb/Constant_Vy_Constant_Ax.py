import numpy as np

class ClimbConstantVyConstantAx():
	"""
	docstring for ClimbConstantVyConstantAx
	kwargs:
		speed_Y			: speed of climb in Y-direction [m/s]
		distance_Y		: distance of the climb in Y-direction [m]
		initial_speed_X : initial speed in X-direction [m/s]
		final_speed_X	: final speed in X-direction [m/s]
		acceleration_X 	: constant acceleration in X-direction [m/s**2]
		distance_X 		: distance of the climb in X-direction [m]
		duration		: total duration of the climb [s]
	"""
	def __init__(self, id:int, name:str, initial_speed_X:float, kwargs:dict, n_discrete=10):
		super(ClimbConstantVyConstantAx, self).__init__()
		self.id = id 								# segment id
		self.name = name 							# segment name
		self.kind = 'ClimbConstantVyConstantAx'		# segment kind
		self.n_discrete = n_discrete				# mission discretization
		self.kwargs = kwargs

		self.AoA = None
		self.speed_Y = None
		self.distance_Y = None
		self.initial_speed_X = initial_speed_X
		self.final_speed_X = None
		self.acceleration_X = None
		self.distance_X = None
		self.duration = None

		self.distance = None # Euclidean distance

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'speed_Y':
				self.speed_Y = float(self.kwargs[item])
			elif item == 'distance_Y':
				self.distance_Y = float(self.kwargs[item])		
			elif item == 'final_speed_X':
				self.final_speed_X = float(self.kwargs[item])
			elif item == 'acceleration_X':
				self.acceleration_X = float(self.kwargs[item])
			elif item == 'distance_X':
				self.distance_X = float(self.kwargs[item])
			elif item == 'duration':
				self.duration = float(self.kwargs[item])
		
		try:
			if self.duration is None:
				self.duration = self.distance_Y / self.speed_Y
			if self.speed_Y is None:
				self.speed_Y = self.distance_Y / self.duration
			if self.distance_Y is None:
				self.distance_Y = self.speed_Y * self.duration
				
		except:
			raise NameError("Need to define at least two of the followings: speed_Y, distance_Y, duration; Must also define final_speed_X;")

		self.acceleration_X = (self.final_speed_X - self.initial_speed_X) / self.duration
		self.distance_X = self.initial_speed_X * self.duration + 0.5 * self.acceleration_X * self.duration**2
		self.distance = np.sqrt(self.distance_X**2 + self.distance_Y**2)

	def _calc_time(self, t_list):
		t0 = t_list[-1][-1]
		t_next = np.linspace(t0, t0+self.duration, self.n_discrete+1)
		return t_next

	def _calc_position(self, x_list, y_list, t_next):
		x0, y0, t0 = x_list[-1][-1], y_list[-1][-1], t_next[0]
		# Position under constant Vy and constant Ax
		x_next = x0 + self.initial_speed_X * (t_next - t0) + 0.5 * self.acceleration_X * (t_next - t0)**2
		y_next = y0 + self.speed_Y * (t_next - t0)
		x_list.append(x_next)
		y_list.append(y_next)
		return x_list, y_list

	def _calc_velocity(self, vx_list, vy_list, t_next):
		vx0, vy0, t0 = vx_list[-1][-1], self.speed_Y, t_next[0]
		# Velocity under constant acceleration in x
		vx_next = vx0 + self.acceleration_X * (t_next - t0)
		vy_next = vy0 + 0.0 * (t_next - t0)
		vx_list.append(vx_next)
		vy_list.append(vy_next)
		return vx_list, vy_list

	def _calc_acceleration(self, ax_list, ay_list, t_next):
		# Zero acceleration in y, and constant acceleration in x
		ax_next = self.acceleration_X * np.ones_like(t_next)
		ay_next = np.zeros_like(t_next)
		ax_list.append(ax_next)
		ay_list.append(ay_next)
		return ax_list, ay_list

	def _info(self):
		info = f'\t\tSegment ID = {self.id}\n'
		info += f'\t\tSegment Name: {self.name}\n'
		info += f'\t\t\tSpeed_Y = {self.speed_Y} m/s\n'
		info += f'\t\t\tDistance_Y = {self.distance_Y} m\n'
		info += f'\t\t\tInitial_speed_X = {self.initial_speed_X} m/s\n'
		info += f'\t\t\tFinal_speed_X = {self.final_speed_X} m/s\n'
		info += f'\t\t\tAcceleration_X = {self.acceleration_X} m/s2\n'
		info += f'\t\t\tDistance_X = {self.distance_X} m\n'
		info += f'\t\t\tDuration = {self.duration} s\n'
		info += f'\t\t\tEuclidean_distance = {self.distance} m'
		return info

	def print_info(self):
		print(f'Segment ID = {self.id}')
		print(f'Segment Name: {self.name}')
		print(f'\tSpeed_Y = {self.speed_Y} m/s')
		print(f'\tDistance_Y = {self.distance_Y} m')
		print(f'\tInitial_speed_X = {self.initial_speed_X} m/s')
		print(f'\tFinal_speed_X = {self.final_speed_X} m/s')
		print(f'\tAcceleration_X = {self.acceleration_X} m/s2')
		print(f'\tDistance_X = {self.distance_X} m')
		print(f'\tDuration = {self.duration} s')
		print(f'\tEuclidean_distance = {self.distance} m')

