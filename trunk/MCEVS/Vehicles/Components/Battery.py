class Battery(object):
	"""
	docstring for Battery
	"""
	def __init__(self, kwargs:dict):
		super(Battery, self).__init__()
		self.kwargs = kwargs
		self.density = None
		self.efficiency = None
		self.max_discharge = None

		# Weight and performance
		self.weight = None
		self.performance = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'density':
				self.density = float(self.kwargs[item])
			elif item == 'efficiency':
				self.efficiency = float(self.kwargs[item])
			elif item == 'max_discharge':
				self.max_discharge = float(self.kwargs[item])

	def _info(self):
		info = '\tComponent name: Battery\n'
		info += f'\t\tDensity = {self.density} Wh/kg\n'
		info += f'\t\tEfficiency = {self.efficiency}\n'
		info += f'\t\tMax discharge = {self.max_discharge}'
		return info

	def print_info(self):
		print('Component name: Battery')
		print(f'\tDensity = {self.density} Wh/kg')
		print(f'\tEfficiency = {self.efficiency}')
		print(f'\tMax discharge = {self.max_discharge}')