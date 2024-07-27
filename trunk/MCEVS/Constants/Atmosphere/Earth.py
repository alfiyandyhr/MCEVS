import numpy as np

class Constant_Temperature(object):
	"""
	docstring for Constant_Temperature
	Assumptions:
		> Data from U.S. Standard Atmosphere (1976 version), but modified temperature (constant)
		> Altitude is from -2 km to 85 km
		> Altitude is divided into 8 sub-layers
		> Constant temperature everywhere
		> Constant gravitational acceleration
	"""
	def __init__(self):
		super(Constant_Temperature, self).__init__()
		self.break_altitudes 	= np.array( [-2000.0,	0.000,		11000.0,	20000.0,	32000.0,	47000.0,	51000.0,	71000.0,	84852.0] )		# m
		self.break_temperatures = np.array( [301.5,		301.5,		301.5,		301.5,		301.5,		301.5,		301.5,		301.5,		301.5] )		# K
		self.break_pressures 	= np.array( [127774.0,	101325.0,	22632.1,	5474.89,	868.019,	110.906,	66.9389, 	3.95642,	0.3734] )		# Pa
		self.break_densities 	= np.array( [1.545586,	1.2256523,	0.273764,	0.0662256,	0.0105000,  1.3415E-3,	8.0971E-4,	4.78579E-5,	4.51674E-6] )	# kg/m**3

		# Constant of air
		self.R 		= 287.0528742 	# Joules/(kgK)
		self.beta 	= 1.458E-6 		# kg/m-s-sqrt(K), Sutherland's constant
		self.gamma  = 1.401 		# ratio of specific heats of air (adiabatic index)
		self.S 		= 110.4 		# K, Sutherland's constant for air

	def compute_constants(self, altitude=0.0, gravity=9.80665):
		"""
		Computes constants at an altitude
		altitude is in m.
		Inputs:
			altitude in m
			gravity  in m/s**2
		Output:
			a dictionary of constants: H, P, T, rho, mu, nu, v_sound
		"""
		# for i in range(len(self.break_altitudes)):
		# 	print(self.break_pressures[i]/self.R/self.break_temperatures[i])

		if altitude < -2000.0 or altitude >84852.0:
			raise ValueError('Altitude is outside the range [-2000,84852] m')

		# Container for constants
		data = {}

		# Check if the altitude is in the breaks altitude
		for i, break_altitude in enumerate(self.break_altitudes):
			if altitude == break_altitude:
				data['H'] 		= float(altitude)
				data['T'] 		= self.break_temperatures[i]
				data['P'] 		= self.break_pressures[i]
				data['rho'] 	= self.break_densities[i]
				data['mu']  	= self.beta * (data['T']**1.5/(data['T']+self.S))
				data['nu']		= data['mu']/data['rho']
				data['v_sound'] = np.sqrt(self.gamma * data['P'] / data['rho'])
				return data

		# Check which sub-layer altitude falls
		for i in range(8):
			if altitude > self.break_altitudes[i] and altitude < self.break_altitudes[i+1]: 
				H0   = self.break_altitudes[i]
				P0   = self.break_pressures[i]
				T0 	 = self.break_temperatures[i]
				data['H']		= float(altitude)
				data['P']		= P0 * np.exp(-gravity/(self.R*T0) * (altitude-H0))
				data['T']		= T0
				data['rho'] 	= data['P'] / (self.R * data['T'])
				data['mu']  	= self.beta * (data['T']**1.5/(data['T']+self.S))
				data['nu']		= data['mu']/data['rho']
				data['v_sound'] = np.sqrt(self.gamma * data['P'] / data['rho'])
				return data

class US_Standard_1976(object):
	"""
	docstring for US_Standard_1976
	Assumptions:
		> Data from U.S. Standard Atmosphere (1976 version)
		> Altitude is from -2 km to 85 km
		> Altitude is divided into 8 sub-layers
		> Constant temperature rate of change at each sub-layer
		> Constant gravitational acceleration
	"""
	def __init__(self):
		super(US_Standard_1976, self).__init__()
		self.break_altitudes 	= np.array( [-2000.0,	0.000,		11000.0,	20000.0,	32000.0,	47000.0,	51000.0,	71000.0,	84852.0] )		# m
		self.break_temperatures = np.array( [301.5,		288.15,		216.65,		216.65,		228.65,		270.65,		270.65,		214.65,		186.95] )		# K
		self.break_pressures 	= np.array( [127774.0,	101325.0,	22632.1,	5474.89,	868.019,	110.906,	66.9389, 	3.95642,	0.3734] )		# Pa
		self.break_densities 	= np.array( [1.47808E0,	1.2250E0,	3.63918E-1,	8.80349E-2,	1.32250E-2, 1.42753E-3,	8.61606E-4,	6.42099E-5,	6.95792E-6] )	# kg/m**3

		self.T_lapse_rate		= np.array( [-0.0067, -0.0065, 0.0000, 0.0010, 0.0028, 0.0000, -0.0028, -0.0020] )	# K or C

		# Constant of air
		self.R 		= 287.0528742 	# Joules/(kgK)
		self.beta 	= 1.458E-6 		# kg/m-s-sqrt(K), Sutherland's constant
		self.gamma  = 1.401 		# ratio of specific heats of air (adiabatic index)
		self.S 		= 110.4 		# K, Sutherland's constant for air

	def compute_constants(self, altitude=0.0, gravity=9.80665):
		"""
		Computes constants at an altitude
		altitude is in m.
		Inputs:
			altitude in m
			gravity  in m/s**2
		Output:
			a dictionary of constants: H, P, T, rho, mu, nu, v_sound
		Source:
			[1] https://www.cfd-online.com/Wiki/Sutherland's_law
		"""

		if altitude < -2000.0 or altitude >84852.0:
			raise ValueError('Altitude is outside the range [-2000,84852] m')

		# Container for constants
		data = {}

		# Check if the altitude is in the breaks altitude
		for i, break_altitude in enumerate(self.break_altitudes):
			if altitude == break_altitude:
				data['H'] 		= float(altitude)
				data['T'] 		= self.break_temperatures[i]
				data['P'] 		= self.break_pressures[i]
				data['rho'] 	= self.break_densities[i]
				data['mu']  	= self.beta * (data['T']**1.5/(data['T']+self.S))
				data['nu']		= data['mu']/data['rho']
				data['v_sound'] = np.sqrt(self.gamma * data['P'] / data['rho'])
				return data

		# Check which sub-layer altitude falls
		for i in range(8):
			if altitude > self.break_altitudes[i] and altitude < self.break_altitudes[i+1]: 
				H0   = self.break_altitudes[i]
				P0   = self.break_pressures[i]
				T0 	 = self.break_temperatures[i]
				rate = self.T_lapse_rate[i]
				data['H']		= float(altitude)
				data['P']		= P0 * ( 1 + (altitude-H0)/T0 * rate)**(-gravity/(self.R*rate))
				data['T']		= T0 + rate * (altitude-H0) 
				data['rho'] 	= data['P'] / (self.R * data['T'])
				data['mu']  	= self.beta * (data['T']**1.5/(data['T']+self.S))
				data['nu']		= data['mu']/data['rho']
				data['v_sound'] = np.sqrt(self.gamma * data['P'] / data['rho'])
				return data

