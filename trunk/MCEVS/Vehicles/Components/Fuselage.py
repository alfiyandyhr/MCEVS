import numpy as np

class Fuselage(object):
	"""
	docstring for Fuselage
	"""
	def __init__(self, kwargs:dict):
		super(Fuselage, self).__init__()
		self.kwargs = kwargs
		self.name = 'fuselage'

		self.length = None
		self.max_diameter = None
		self.number_of_passenger = None
		self.fineness_ratio = None
		self.weight = None

		# Use of new material that reduces weight?
		self.technology_factor = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'length':
				self.length = float(self.kwargs[item])
			elif item == 'max_diameter':
				self.max_diameter = float(self.kwargs[item])
			elif item == 'number_of_passenger':
				self.number_of_passenger = int(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

		self.fineness_ratio = self.length / self.max_diameter

	def _calculate_weight_given_mtow(self, mtow):
		W_fuselage = 14.86 * (mtow*2.20462)**0.144
		W_fuselage *= (self.length/(np.pi * self.max_diameter))**0.778
		W_fuselage *= (self.length*3.28084)**0.383
		W_fuselage *= self.number_of_passenger**0.455
		self.weight = W_fuselage * 0.453592 * self.technology_factor

	def _info(self):
		info = '\tComponent name: Fuselage\n'
		info += f'\t\tLength = {self.length} m\n'
		info += f'\t\tMax diameter = {self.max_diameter} m\n'
		info += f'\t\tNumber of passenger = {self.number_of_passenger}'
		return info

	def print_info(self):
		print('Component name: Fuselage')
		print(f'\tLength = {self.length} m')
		print(f'\tMax diameter = {self.max_diameter} m')
		print(f'\tNumber of passenger = {self.number_of_passenger}')