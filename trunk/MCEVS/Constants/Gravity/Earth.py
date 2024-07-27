class EarthGravityModel(object):
	"""
	docstring for EarthGravityModel
	"""
	def __init__(self):
		super(EarthGravityModel, self).__init__()
		self.mean_radius = 6.371E6 			# m
		self.sea_level_gravity = 9.80665 	# m/s**2

	def compute_gravity(self, altitude:float):
		"""
		Compute the gravitational acceleration at an altitude
		Altitude is in "m"
		"""
		g0 = self.sea_level_gravity
		Re = self.mean_radius

		gh = g0 * (Re / (Re+altitude))**2

		return gh
