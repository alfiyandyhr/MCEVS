import numpy as np

class Rotor(object):
	"""
	Rotor object
	Notes:
		> should be called with "initialize_rotor" function
	"""
	def __init__(self, rotorDict:dict, caseDict:dict):
		super(Rotor, self).__init__()
		
		# Unpacking
		D = rotorDict['diameter']
		rpm = caseDict['rpm']
		v_inf = caseDict['v_inf']

		self.nblades = rotorDict['nblades']
		self.blade_radius = 0.5 * D
		self.hub_radius = rotorDict['hub_radius']
		self.area = 0.25 * np.pi * D**2

		# Operating point
		self.op = {'v_inf': v_inf,
				   'omega': rpm*2*np.pi/60.0,
				   'n': rpm/60.0,
				   'J': v_inf*60.0/(D*rpm)}

	def _calc_performance(self, T, Q, P, rho):
		"""
		Calculate rotor performance given thrust, torque, power
		"""
		# Unpacking
		n = self.op['n']
		D = 2.0 * self.blade_radius
		J = self.op['J']

		# Ideal power
		ideal_P = np.sqrt(T**3 / (2.0 * rho * self.area))

		# Figure of merit
		FM = ideal_P / P

		# Coefficients
		CT = T / (rho * n**2 * D**4)
		CQ = Q / (rho * n**2 * D**5)
		CP = 2.0 * np.pi * CQ
		eta = CT / CP * J

		return FM, CT, CQ, CP, eta

def initialize_rotor(rotorDict, caseDict):
	r = Rotor(rotorDict, caseDict)
	return r