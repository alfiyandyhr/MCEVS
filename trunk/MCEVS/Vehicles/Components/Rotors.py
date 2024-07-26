class LiftRotor(object):
	"""
	docstring for LiftRotor
	"""
	def __init__(self, kwargs:dict):
		super(LiftRotor, self).__init__()
		self.kwargs = kwargs

		# Geometric information
		self.n_rotor = None
		self.solidity = None
		self.radius = None

		# Weight and performance
		self.weight = None
		self.performance = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'n_rotor':
				self.n_rotor = int(self.kwargs[item])
			elif item == 'solidity':
				self.solidity = float(self.kwargs[item])
			elif item == 'radius':
				self.radius = float(self.kwargs[item])

	def _info(self):
		info = '\tComponent name: Lift Rotor\n'
		info += f'\t\tNumber of rotor(s) = {self.n_rotor}\n'
		info += f'\t\tSolidity = {self.solidity}\n'
		info += f'\t\tRadius = {self.radius} m'
		return info

	def print_info(self):
		print('Component name: Lift Rotor')
		print(f'\tNumber of rotor(s) = {self.n_rotor}')
		print(f'\tSolidity = {self.solidity}')
		print(f'\tRadius = {self.radius} m')

class Propeller(object):
	"""
	docstring for Propeller
	"""
	def __init__(self, kwargs:dict):
		super(Propeller, self).__init__()
		self.kwargs = kwargs

		# Geometric information
		self.n_propeller = None
		self.solidity = None
		self.radius = None

		# Weight and performance
		self.weight = None
		self.performance = None
	
	def _initialize(self):
		for item in list(self.kwargs):
			if item == 'n_propeller':
				self.n_propeller = int(self.kwargs[item])
			elif item == 'solidity':
				self.solidity = float(self.kwargs[item])
			elif item == 'radius':
				self.radius = float(self.kwargs[item])

	def _info(self):
		info = '\tComponent name: Propeller\n'
		info += f'\t\tNumber of propeller(s) = {self.n_propeller}\n'
		info += f'\t\tSolidity = {self.solidity}\n'
		info += f'\t\tRadius = {self.radius} m'
		return info

	def print_info(self):
		print('Component name: Propeller')
		print(f'\tNumber of propeller(s) = {self.n_propeller} m')
		print(f'\tSolidity = {self.solidity} m')
		print(f'\tRadius = {self.radius} m')






