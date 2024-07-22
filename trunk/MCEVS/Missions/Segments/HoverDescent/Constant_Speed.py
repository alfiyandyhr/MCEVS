class HoverDescentConstantSpeed():
	"""
	docstring for HoverDescentConstantSpeed
	kwargs:
		speed 		: constant hover climb speed [m/s]
		distance 	: distance of the hover climb [m]
		duration	: duration of the hover climb [s]
	"""
	def __init__(self, id:int, name:str, kwargs:dict):
		super(HoverDescentConstantSpeed, self).__init__()
		self.id = id 		# segment id
		self.name = name 	# segment name
		self.kwargs = kwargs
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

