class Airfoil(object):
	"""
	Represents an airfoil with basic aerodynamic properties.
	"""
	_airfoil_data = {
		'NACA2412': {'CL_alpha': 4.0923, 'CL_0': 0.415},
		'NACA0012': {'CL_alpha': 5.4225, 'CL_0': 0.0},
	}

	def __init__(self, name):
		self.name = name.upper()

	@property
	def CL_alpha(self):
		try:
			return self._airfoil_data[self.name]['CL_alpha']
		except KeyError:
			raise ValueError(f"Airfoil '{self.name}' not recognized.")

	@property
	def CL_0(self):
		try:
			return self._airfoil_data[self.name]['CL_0']
		except KeyError:
			raise ValueError(f"Airfoil '{self.name}' not recognized.")

	def __repr__(self):
		return f"<Airfoil {self.name}: CL_alpha={self.CL_alpha}, CL_0={self.CL_0}>"