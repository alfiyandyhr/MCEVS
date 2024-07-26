class Fuselage(object):
	"""
	docstring for Fuselage
	"""
	def __init__(self, kwargs:dict):
		super(Fuselage, self).__init__()
		self.kwargs = kwargs
		self.length = None
		self.max_perimeter = None
		self.weight = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'length':
				self.length = float(self.kwargs[item])
			elif item == 'max_perimeter':
				self.max_perimeter = float(self.kwargs[item])

	def _info(self):
		info = '\tComponent name: Fuselage\n'
		info += f'\t\tLength = {self.length} m\n'
		info += f'\t\tMax perimeter = {self.max_perimeter} m'
		return info

	def print_info(self):
		print('Component name: Fuselage')
		print(f'\tLength = {self.length} m')
		print(f'\tMax perimeter = {self.max_perimeter} m')