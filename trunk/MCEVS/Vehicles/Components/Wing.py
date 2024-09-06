class Wing(object):
	"""
	docstring for Wing
	"""
	def __init__(self, kwargs:dict):
		super(Wing, self).__init__()
		self.name = 'wing'
		self.kwargs = kwargs

		# Geometric information
		self.area = None
		self.span = None
		self.aspect_ratio = None
		self.sweep_angle = None
		self.dihedral = None
		self.taper_ratio = None
		self.root_chord_length = None
		self.tip_chord_length = None
		self.mean_chord_length = None
		self.ultimate_load_factor = None
		self.thickness_to_chord_ratio = None

		# Weight and performance
		self.weight = None
		self.performance = None

		# Use of new material that reduces weight?
		self.technology_factor = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'area':
				self.area = float(self.kwargs[item])
			elif item == 'aspect_ratio':
				self.aspect_ratio = float(self.kwargs[item])
			elif item == 'ultimate_load_factor':
				self.ultimate_load_factor = float(self.kwargs[item])
			elif item == 'thickness_to_chord_ratio':
				self.thickness_to_chord_ratio = float(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_mtow(self, mtow):
		W_wing = 0.04674 * (mtow*2.20462)**0.397
		W_wing *= (self.area*3.28084*3.28084)**0.360
		W_wing *= (self.ultimate_load_factor)**0.397 * (self.aspect_ratio)**1.712
		self.weight = W_wing * 0.453592 * self.technology_factor

	def _info(self):
		info = '\tComponent name: Wing\n'
		info += f'\t\tAspect ratio = {self.aspect_ratio}\n'
		info += f'\t\tArea = {self.area} m**2\n'
		info += f'\t\tUltimate load factor = {self.ultimate_load_factor}'
		return info

	def print_info(self):
		print('Component name: Wing')
		print(f'\tAspect ratio = {self.aspect_ratio}')
		print(f'\tArea = {self.area} m**2')
		print(f'\tUltimate load factor = {self.ultimate_load_factor}')