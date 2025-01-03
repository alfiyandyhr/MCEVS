import openmdao.api as om
import numpy as np

class Rotor(object):
	"""
	Rotor object
	Notes:
		> should be called with "initialize_rotor" function
	"""
	def __init__(self, rotorDict:dict):
		super(Rotor, self).__init__()
		
		# Unpacking
		D = rotorDict['diameter']

		self.diameter = rotorDict['diameter']
		self.nblades = rotorDict['nblades']
		self.blade_radius = 0.5 * rotorDict['diameter']
		self.hub_radius = rotorDict['hub_radius']
		self.area = 0.25 * np.pi * rotorDict['diameter']**2

		# Operating point
		self.op = {'v_inf': None,
				   'rpm': None,
				   'omega': None,
				   'n': None,
				   'J': None}

	def _calc_performance(self, T, Q, P, rho):
		"""
		Calculate rotor performance given thrust, torque, power
		"""
		# Unpacking
		D = self.diameter
		v_inf = self.op['v_inf']
		n = self.op['n']
		J = self.op['J']

		# Ideal power
		ideal_P = T * ( v_inf/2 + np.sqrt((v_inf/2)**2 + T/(2*rho*self.area)) )

		# Figure of merit
		FM = ideal_P / P

		# Coefficients
		CT = T / (rho * n**2 * D**4)
		CQ = Q / (rho * n**2 * D**5)
		CP = 2.0 * np.pi * CQ
		eta = CT / CP * J

		return FM, CT, CQ, CP, eta

def initialize_rotor(rotorDict):
	r = Rotor(rotorDict)
	return r

class RotorPerformanceCoeffs(om.ExplicitComponent):
	"""
	Parameter: rho
	Inputs: T, Q, P, v_inf, omega, blade_radius
	Outputs: FM, CT, CQ, CP, eta, J
	"""
	def initialize(self):
		self.options.declare('rho', types=float)

	def setup(self):
		self.add_input('T', units='N')
		self.add_input('Q', units='N*m')
		self.add_input('P', units='W')
		self.add_input('v_inf', units='m/s')
		self.add_input('omega', units='rad/s')
		self.add_input('blade_radius', units='m')
		self.add_output('CT', units=None)
		self.add_output('CP', units=None)
		self.add_output('CQ', units=None)
		self.add_output('eta', units=None)
		self.add_output('FM', units=None)
		self.add_output('J', units=None)
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		rho = self.options['rho']
		T = inputs['T']
		Q = inputs['Q']
		P = inputs['P']
		v_inf = inputs['v_inf']
		omega = inputs['omega']
		blade_radius = inputs['blade_radius']

		n = omega /(2*np.pi) # rev/s
		J = v_inf/(2*n*blade_radius)

		ideal_P = T * ( v_inf/2 + np.sqrt((v_inf/2)**2 + T/(2*rho*np.pi*blade_radius**2)) )
		
		outputs['CT'] = T / (16 * rho * n**2 * blade_radius**4)
		outputs['CQ'] = Q / (32 * rho * n**2 * blade_radius**5)
		outputs['CP'] = 2 * np.pi * Q / (32 * rho * n**2 * blade_radius**5)
		outputs['eta'] = T * blade_radius * J / (np.pi * Q)
		outputs['FM'] = ideal_P / P
		outputs['J'] = J

	def compute_partials(self, inputs, partials):
		rho = self.options['rho']
		T = inputs['T']
		Q = inputs['Q']
		P = inputs['P']
		v_inf = inputs['v_inf']
		omega = inputs['omega']
		blade_radius = inputs['blade_radius']

		n = omega /(2*np.pi) # rev/s
		J = v_inf/(2*n*blade_radius)
		dn_domega = 1/(2*np.pi)

		v_ind = np.sqrt( (v_inf/2)**2 + T/(2*rho*np.pi*blade_radius**2) )
		dvind_dT = 0.5 / v_ind * (1/(2*rho*np.pi*blade_radius**2))
		dvind_dvinf = 0.5 / v_ind * (0.5*v_inf)
		dvind_dr = 0.5 / v_ind * (T/(2*rho*np.pi)) * (-2/blade_radius**3)

		ideal_P = T * v_inf/2 + T * v_ind

		dIP_dT = v_inf/2 + v_ind + dvind_dT
		dIP_dvinf = T/2 + T * dvind_dvinf
		dIP_dr = T * dvind_dr
		
		partials['CT','T'] = 1.0 / (16 * rho * n**2 * blade_radius**4)
		partials['CT','Q'] = 0.0
		partials['CT','P'] = 0.0
		partials['CT','v_inf'] = 0.0
		partials['CT','omega'] = T / (16 * rho * blade_radius**4) * (-2/n**3) * dn_domega
		partials['CT','blade_radius'] = T / (16 * rho * n**2) * (-4/blade_radius**5)

		partials['CQ','T'] = 0.0
		partials['CQ','Q'] = 1.0 / (32 * rho * n**2 * blade_radius**5)
		partials['CQ','P'] = 0.0
		partials['CQ','v_inf'] = 0.0
		partials['CQ','omega'] = Q / (32 * rho * blade_radius**5) * (-2/n**3) * dn_domega
		partials['CQ','blade_radius'] = Q / (32 * rho * n**2) * (-5/blade_radius**6)

		partials['CP','T'] = 0.0
		partials['CP','Q'] = 2 * np.pi / (32 * rho * n**2 * blade_radius**5)
		partials['CP','P'] = 0.0
		partials['CP','v_inf'] = 0.0
		partials['CP','omega'] = 2 * np.pi * Q / (32 * rho * blade_radius**5) * (-2/n**3) * dn_domega
		partials['CP','blade_radius'] = 2 * np.pi * Q / (32 * rho * n**2) * (-5/blade_radius**6)

		partials['eta','T'] = v_inf / (omega * Q)
		partials['eta','Q'] = T * v_inf / omega * (-1/Q**2)
		partials['eta','P'] = 0.0
		partials['eta','v_inf'] = T / (omega * Q)
		partials['eta','omega'] = T * v_inf / Q * (-1/omega**2)
		partials['eta','blade_radius'] = 0.0

		partials['FM','T'] = dIP_dT / P
		partials['FM','Q'] = 0.0
		partials['FM','P'] = ideal_P * (-1/P**2)
		partials['FM','v_inf'] = dIP_dvinf / P
		partials['FM','omega'] = 0.0
		partials['FM','blade_radius'] = dIP_dr / P

		partials['J','T'] = 0.0
		partials['J','Q'] = 0.0
		partials['J','P'] = 0.0
		partials['J','v_inf'] = np.pi / (omega * blade_radius)
		partials['J','omega'] = v_inf * np.pi / blade_radius * (-1/omega**2)
		partials['J','blade_radius'] = v_inf * np.pi / omega * (-1/blade_radius**2)







		







