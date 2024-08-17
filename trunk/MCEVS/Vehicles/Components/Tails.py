import numpy as np

class HorizontalTail(object):
	"""
	docstring for HorizontalTail
	"""
	def __init__(self, kwargs:dict):
		super(HorizontalTail, self).__init__()
		self.name = 'HorizontalTail'
		self.kwargs = kwargs

		# Geometric information
		self.area = None
		self.span = None
		self.aspect_ratio = None
		self.sweep_angle = None # at quarter chord
		self.dihedral = None
		self.taper_ratio = None
		self.root_chord_length = None
		self.tip_chord_length = None
		self.mean_chord_length = None
		self.max_root_thickness = None

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
			elif item == 'taper_ratio':
				self.taper_ratio = float(self.kwargs[item])
			elif item == 'max_root_thickness':
				self.max_root_thickness = float(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_mtow(self, mtow):
		W_HTail = 3.184 * (mtow*2.20462)**0.887
		W_HTail *= (self.area*3.28084*3.28084)**0.101
		W_HTail *= self.aspect_ratio**0.138
		W_HTail /= (174.04 * (self.max_root_thickness*3.28084)**0.223)
		self.weight = W_HTail * 0.453592 * self.technology_factor

	def _info(self):
		info  = '\tComponent name: Horizontal Tail\n'
		info += f'\t\tAspect ratio = {self.aspect_ratio}\n'
		info += f'\t\tArea = {self.area} m**2\n'
		info += f'\t\tMax root thickness = {self.max_root_thickness} m'
		return info

	def print_info(self):
		print('Component name: Horizontal Tail')
		print(f'\tAspect ratio = {self.aspect_ratio}')
		print(f'\tArea = {self.area} m**2')
		print(f'\tMax root thickness = {self.max_root_thickness} m')

class VerticalTail(object):
	"""
	docstring for VerticalTail
	"""
	def __init__(self, kwargs:dict):
		super(VerticalTail, self).__init__()
		self.name = 'VerticalTail'
		self.kwargs = kwargs

		# Geometric information
		self.area = None
		self.span = None
		self.aspect_ratio = None
		self.sweep_angle = None # at quarter chord
		self.dihedral = None
		self.taper_ratio = None
		self.root_chord_length = None
		self.tip_chord_length = None
		self.mean_chord_length = None
		self.max_root_thickness = None

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
			elif item == 'taper_ratio':
				self.taper_ratio = float(self.kwargs[item])
			elif item == 'max_root_thickness':
				self.max_root_thickness = float(self.kwargs[item])
			elif item == 'sweep_angle':
				self.sweep_angle = float(self.kwargs[item])	
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_mtow(self, mtow):
		W_VTail = 1.68 * (mtow*2.20462)**0.567
		W_VTail *= (self.area*3.28084*3.28084)**1.249
		W_VTail *= self.aspect_ratio**0.482
		W_VTail /= (639.5 * (self.max_root_thickness*3.28084)**0.747)
		W_VTail /= (np.cos(self.sweep_angle/180*np.pi))**0.882
		self.weight = W_VTail * 0.453592 * self.technology_factor

	def _info(self):
		info  = '\tComponent name: Vertical Tail\n'
		info += f'\t\tAspect ratio = {self.aspect_ratio}\n'
		info += f'\t\tArea = {self.area} m**2\n'
		info += f'\t\tMax root thickness = {self.max_root_thickness} m\n'
		info += f'\t\tQuarter chord sweep angle = {self.sweep_angle} deg'
		return info

	def print_info(self):
		print('Component name: Vertical Tail')
		print(f'\tAspect ratio = {self.aspect_ratio}')
		print(f'\tArea = {self.area} m**2')
		print(f'\tMax root thickness = {self.max_root_thickness} m')
		print(f'\tQuarter chord sweep angle = {self.sweep_angle} deg')