import numpy as np

class Boom(object):
	"""
	docstring for Boom
	"""
	def __init__(self, kwargs:dict):
		super(Boom, self).__init__()
		self.kwargs = kwargs
		self.name = 'boom'

		# Represented as a fuselage/pod
		self.length = None
		self.max_diameter = None
		self.fineness_ratio = None

		# Represented as a wing
		self.thickness_to_chord_ratio = None
		self.area = None
		self.aspect_ratio = None

		self.number_of_booms = None
		self.weight = None

		# Use of new material that reduces weight?
		self.technology_factor = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'length':
				self.length = float(self.kwargs[item])
			elif item == 'max_diameter':
				self.max_diameter = float(self.kwargs[item])
			elif item == 'number_of_booms':
				self.number_of_booms = int(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])
			elif item == 'fineness_ratio':
				if len(self.kwargs[item]) == 1:
					self.fineness_ratio = float(self.kwargs[item])
				else:
					self.fineness_ratio = self.kwargs[item]
			elif item == 'thickness_to_chord_ratio':
				if type(self.kwargs[item]) == list:
					self.thickness_to_chord_ratio = self.kwargs[item]
				else:
					self.thickness_to_chord_ratio = float(self.kwargs[item])

		if self.fineness_ratio is None and self.length is not None and self.max_diameter is not None:
			self.fineness_ratio = self.length / self.max_diameter

	def _calculate_weight_given_mtow(self, mtow):
		pass

	def _info(self):
		info = '\tComponent name: Boom\n'
		info += f'\t\tLength = {self.length} m\n'
		info += f'\t\tMax diameter = {self.max_diameter} m\n'
		info += f'\t\tNumber of booms = {self.number_of_booms}'
		return info

	def print_info(self):
		print('Component name: Boom')
		print(f'\tLength = {self.length} m')
		print(f'\tMax diameter = {self.max_diameter} m')
		print(f'\tNumber of booms = {self.number_of_booms}')