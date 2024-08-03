class LiftRotor(object):
	"""
	docstring for LiftRotor
	"""
	def __init__(self, kwargs:dict):
		super(LiftRotor, self).__init__()
		self.kwargs = kwargs
		self.name = 'lift_rotor'

		# Geometric information
		self.n_rotor = None
		self.n_blade = None
		self.solidity = None
		self.radius = None

		# Weight and performance
		self.weight = None
		self.figure_of_merit = None
		self.advance_ratio = None
	
		# Use of new material that reduces weight?
		self.technology_factor = None

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'n_rotor':
				self.n_rotor = int(self.kwargs[item])
			elif item == 'n_blade':
				self.n_blade = int(self.kwargs[item])
			elif item == 'solidity':
				self.solidity = float(self.kwargs[item])
			elif item == 'radius':
				self.radius = float(self.kwargs[item])
			elif item == 'figure_of_merit':
				self.figure_of_merit = float(self.kwargs[item])
			elif item == 'advance_ratio':
				self.advance_ratio = float(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_max_power(self, p_max):
		k_prop = 0.144 # for piston engines; 0.108 for turboprops
		W_rotor = k_prop * (2*self.radius*3.28084)**0.782
		W_rotor *= (p_max*0.00134101/self.n_rotor)**0.782
		W_rotor *= (self.n_blade**0.5)**0.782
		self.weight = self.n_rotor * W_rotor * 0.453592 * self.technology_factor

	def _info(self):
		info = '\tComponent name: Lift Rotor\n'
		info += f'\t\tNumber of rotor(s) = {self.n_rotor}\n'
		info += f'\t\tNumber of blade(s) per rotor = {self.n_blade}\n'
		info += f'\t\tSolidity = {self.solidity}\n'
		info += f'\t\tRadius = {self.radius} m\n'
		info += f'\t\tFigure of Merit = {self.figure_of_merit}\n'
		info += f'\t\tAdvance ratio = {self.advance_ratio}'
		return info

	def print_info(self):
		print('Component name: Lift Rotor')
		print(f'\tNumber of rotor(s) = {self.n_rotor}')
		print(f'\tNumber of blade(s) per rotor = {self.n_blade}')
		print(f'\tSolidity = {self.solidity}')
		print(f'\tRadius = {self.radius} m')
		print(f'\tFigure of Merit = {self.figure_of_merit}')
		print(f'\tAdvance ratio = {self.advance_ratio}')

class Propeller(object):
	"""
	docstring for Propeller
	"""
	def __init__(self, kwargs:dict):
		super(Propeller, self).__init__()
		self.kwargs = kwargs
		self.name = 'propeller'

		# Geometric information
		self.n_propeller = None
		self.n_blade = None
		self.solidity = None
		self.radius = None

		# Weight and performance
		self.weight = None
		self.advance_ratio = None
	
		# Use of new material that reduces weight?
		self.technology_factor = None	

	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'n_propeller':
				self.n_propeller = int(self.kwargs[item])
			elif item == 'n_blade':
				self.n_blade = int(self.kwargs[item])
			elif item == 'solidity':
				self.solidity = float(self.kwargs[item])
			elif item == 'radius':
				self.radius = float(self.kwargs[item])
			elif item == 'advance_ratio':
				self.advance_ratio = float(self.kwargs[item])
			elif item == 'technology_factor':
				self.technology_factor = float(self.kwargs[item])

	def _calculate_weight_given_max_power(self, p_max):
		k_prop = 0.144 # for piston engines; 0.108 for turboprops
		W_prop = k_prop * (2*self.radius*3.28084)**0.782
		W_prop *= (p_max*0.00134101/self.n_propeller)**0.782
		W_prop *= (self.n_blade**0.5)**0.782
		self.weight = self.n_propeller * W_prop * 0.453592 * self.technology_factor

	def _info(self):
		info = '\tComponent name: Propeller\n'
		info += f'\t\tNumber of propeller(s) = {self.n_propeller}\n'
		info += f'\t\tNumber of blade(s) per propeller = {self.n_blade}\n'
		info += f'\t\tSolidity = {self.solidity}\n'
		info += f'\t\tRadius = {self.radius} m\n'
		info += f'\t\tAdvance ratio = {self.advance_ratio}'
		return info

	def print_info(self):
		print('Component name: Propeller')
		print(f'\tNumber of propeller(s) = {self.n_propeller} m')
		print(f'\tNumber of blade(s) per propeller = {self.n_blade}')
		print(f'\tSolidity = {self.solidity} m')
		print(f'\tRadius = {self.radius} m')
		print(f'\tAdvance ratio = {self.advance_ratio}')






