class LandingGear(object):
	"""
	docstring for LandingGear
	"""
	def __init__(self, kwargs:dict):
		super(LandingGear, self).__init__()
		self.kwargs = kwargs
		self.name = 'landing_gear'
		self.gear_type = None # either "wheeled" or "skid"

		# for wheeled
		self.strut_length = None
		self.ultimate_load_factor = None

		# for skid
		self.skid_length = None
		self.skid_heights = None

		self.weight = None

		# Use of new material that reduces weight?
		self.technology_factor = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'strut_length':
				self.strut_length = float(self.kwargs[item])
			elif item == 'ultimate_load_factor':
				self.ultimate_load_factor = float(self.kwargs[item])
			elif item == 'skid_length':
				self.skid_length = float(self.kwargs[item])
			elif item == 'skid_heights':
				self.skid_heights = self.kwargs[item]
			elif item == 'gear_type':
				self.gear_type = self.kwargs[item]
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_mtow(self, mtow):

		if self.gear_type == 'wheeled': f_LG = 0.0325
		elif self.gear_type == 'skid': f_LG = 0.014
		else: raise ValueError('Gear type should be either "wheeled" or "skid".')

		W_landing_gear = f_LG * mtow

		self.weight = W_landing_gear * 0.453592 * self.technology_factor

	def _info(self):
		info = '\tComponent name: Landing Gear\n'
		info += f'\t\tGear type = {self.gear_type}\n'
		if self.gear_type == 'wheeled':
			info += f'\t\tStrut length = {self.strut_length} m\n'
			info += f'\t\tUltimate load factor = {self.ultimate_load_factor}'
		elif self.gear_type == 'skid':
			info += f'\t\tSkid length = {self.skid_length} m\n'
			info += f'\t\tSkid heights = {self.skid_heights} m'			
		return info

	def print_info(self):
		print('Component name: Landing Gear')
		print(f'\tGear type = {self.gear_type} m')
		if self.gear_type == 'wheeled':
			print(f'\tStrut length = {self.strut_length} m')
			print(f'\tUltimate load factor = {self.ultimate_load_factor}')
		elif self.gear_type == 'skid':
			print(f'\tSkid length = {self.skid_length} m')
			print(f'\tSkid heights = {self.skid_heights}')