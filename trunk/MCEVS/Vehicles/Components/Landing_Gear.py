class LandingGear(object):
	"""
	docstring for LandingGear
	"""
	def __init__(self, kwargs:dict):
		super(LandingGear, self).__init__()
		self.kwargs = kwargs
		self.strut_length = None
		self.ultimate_load_factor = None
		self.mass_properties = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'strut_length':
				self.strut_length = float(self.kwargs[item])
			elif item == 'ultimate_load_factor':
				self.ultimate_load_factor = float(self.kwargs[item])

	def _info(self):
		info = '\tComponent name: Landing Gear\n'
		info += f'\t\tStrut length = {self.strut_length} m\n'
		info += f'\t\tUltimate load factor = {self.ultimate_load_factor}'
		return info

	def print_info(self):
		print('Component name: Landing Gear')
		print(f'\tStrut length = {self.strut_length} m')
		print(f'\tUltimate load factor = {self.ultimate_load_factor}')