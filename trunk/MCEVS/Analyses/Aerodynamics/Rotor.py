import numpy as np
import openmdao.api as om

class ThrustCoefficient(om.ExplicitComponent):
	"""
	Computes the thrust coefficient of a rotor
		Ct = thrust / (rho * S_disk * V_tip**2)
	Parameter:
		rho_air 		: air density [kg/m**3]
	Inputs:
		Rotor|Thrust 	: thrust of a rotor [N]
		Rotor|radius	: rotor radius [m]
		Rotor|omega 	: rotor's angular velocity [rad/s]
	Outputs:
		Rotor|Ct 		: rotor's thrust coefficient
	Source:
		Johnson, W., “Rotorcraft Aeromechanics,” Cambridge University Press, 2013.
	"""
	def initialize(self):
		self.options.declare('rho_air', default=1.225, desc='Air density')

	def setup(self):
		self.add_input('Rotor|Thrust', units='N', desc='Thrust of a rotor')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.add_output('Rotor|Ct', desc='Thrust coefficient')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|Thrust']
		r = inputs['Rotor|radius']
		omega = inputs['Rotor|omega']

		S_disk = np.pi * r**2
		Vtip = omega * r

		# Calculate Ct
		outputs['Rotor|Ct'] = thrust/(rho_air * S_disk * Vtip**2)

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|Thrust']
		r = inputs['Rotor|radius']
		omega = inputs['Rotor|omega']

		partials['Rotor|Ct', 'Rotor|Thrust'] = 1 / (np.pi * rho_air * omega**2 * r**4)
		partials['Rotor|Ct', 'Rotor|omega'] = thrust/(np.pi*rho_air*r**4) * (-2/omega**3)
		partials['Rotor|Ct', 'Rotor|radius'] = thrust/(np.pi*rho_air*omega**2) * (-4/r**5)

class RotorAdvanceRatio(om.ExplicitComponent):
	"""
	Computes the rotor advance ratio
		mu = V cos(alpha) / (omega * r)
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|alpha		: rotor tilt angle [rad]
		Rotor|omega 	: rotor's angular velocity [rad/s]
		v_inf			: freestream velocity [m/s]
	Outputs:
		Rotor|mu 		: rotor's advance ratio
	Source:
		Johnson, W., “Rotorcraft Aeromechanics,” Cambridge University Press, 2013.
	"""
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle')
		self.add_input('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.add_output('Rotor|mu', desc='Advance ratio of rotor')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		v_inf = inputs['v_inf']
		a = inputs['Rotor|alpha']
		r = inputs['Rotor|radius']
		omega = inputs['Rotor|omega']

		outputs['Rotor|mu'] = v_inf * np.cos(a) / (omega * r)

	def compute_partials(self, inputs, partials):
		v_inf = inputs['v_inf']
		a = inputs['Rotor|alpha']
		r = inputs['Rotor|radius']
		omega = inputs['Rotor|omega']

		partials['Rotor|mu', 'v_inf'] = np.cos(a) / (omega * r)
		partials['Rotor|mu', 'Rotor|alpha'] = - v_inf * np.sin(a) / (omega * r) 
		partials['Rotor|mu', 'Rotor|omega'] = - v_inf * np.cos(a) / (omega**2 * r)
		partials['Rotor|mu', 'Rotor|radius'] = - v_inf * np.cos(a) / (omega * r**2)

class InducedVelocity(om.ExplicitComponent):
	"""
	Computes the induced velocity of a rotor from lambda's equation
		lambda = (v_inf * sin(alpha) + v_ind) / (omega * r)
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|alpha		: rotor tilt angle [rad]
		Rotor|omega 	: rotor's angular velocity [rad/s]
		Rotor|lambda	: rotor inflow ratio, positive down through disk
		v_inf			: freestream velocity [m/s]
	Outputs:
		v_induced		: induced velocity of a rotor [m/s] 
	Source:
		Johnson, W., “Rotorcraft Aeromechanics,” Cambridge University Press, 2013.
	"""
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle')
		self.add_input('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.add_input('Rotor|lambda', desc='Rotor inflow')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_output('v_induced', units='m/s', desc='Induced velocity')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		r = inputs['Rotor|radius']
		a = inputs['Rotor|alpha']
		omega = inputs['Rotor|omega']
		lmbd = inputs['Rotor|lambda']
		v_inf = inputs['v_inf']

		outputs['v_induced'] = omega * r * lmbd - v_inf * np.sin(a)

	def compute_partials(self, inputs, partials):
		r = inputs['Rotor|radius']
		a = inputs['Rotor|alpha']
		omega = inputs['Rotor|omega']
		lmbd = inputs['Rotor|lambda']
		v_inf = inputs['v_inf']

		partials['v_induced', 'Rotor|radius'] = omega * lmbd
		partials['v_induced', 'Rotor|alpha'] = - v_inf * np.cos(a)
		partials['v_induced', 'Rotor|omega'] = r * lmbd
		partials['v_induced', 'Rotor|lambda'] = omega * r
		partials['v_induced', 'v_inf'] = - np.sin(a)

class RotorInflow(om.ImplicitComponent):
	"""
	Computes the inflow of a rotor (lambda) implicitly
	Inputs:
		Rotor|mu 	 : rotor's advance ratio
		Rotor|alpha	 : rotor tilt angle [rad]
		Rotor|Ct 	 : rotor's thrust coefficient
	Outputs:
		Rotor|lambda : rotor inflow ratio, positive down through disk
	Source:
		B. Govindarajan and A. Sridharan, “Conceptual Sizing of Vertical Lift Package Delivery Platforms,”
		Journal of Aircraft, vol. 57, no. 6, pp. 1170–1188, Nov. 2020, doi: 10.2514/1.C035805.
	"""
	def setup(self):
		self.add_input('Rotor|mu', desc='Advance ratio')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle')
		self.add_input('Rotor|Ct', desc='Thrust coefficient')
		self.add_output('Rotor|lambda', val=0.1, lower=0.0, upper=10.0, desc='Rotor inflow')
		self.declare_partials('*', '*')

	def apply_nonlinear(self, inputs, outputs, residuals):
		mu = inputs['Rotor|mu']
		a = inputs['Rotor|alpha']
		Ct = inputs['Rotor|Ct']
		lmbd = outputs['Rotor|lambda']
		# Compute residuals
		residuals['Rotor|lambda'] = mu*np.tan(a) + Ct / (2*np.sqrt((mu**2 + lmbd**2))) - lmbd

	def linearize(self, inputs, outputs, partials):
		mu = inputs['Rotor|mu']
		a = inputs['Rotor|alpha']
		Ct = inputs['Rotor|Ct']
		lmbd = outputs['Rotor|lambda']

		partials['Rotor|lambda', 'Rotor|mu'] = np.tan(a) - (Ct*mu)/(2*np.sqrt((mu**2 + lmbd**2)**3))
		partials['Rotor|lambda', 'Rotor|alpha'] = mu / (np.cos(a) * np.cos(a))
		partials['Rotor|lambda', 'Rotor|Ct'] = 1 / ( 2 * np.sqrt(mu**2 + lmbd**2) )
		partials['Rotor|lambda', 'Rotor|lambda'] = - (Ct*lmbd) / (2 * np.sqrt((mu**2 + lmbd**2)**3)) - 1

class RotorRevolutionFromAdvanceRatio(om.ExplicitComponent):
	"""
	Computes the rotor revolution (omega) given the advance ratio
		omega = V cos(alpha) / (mu * r)
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|alpha		: rotor tilt angle [rad]
		Rotor|mu 	 	: rotor's advance ratio
		v_inf			: freestream velocity [m/s]
	Outputs:
		Rotor|omega 	: rotor's angular velocity [rad/s]
	"""
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle')
		self.add_input('Rotor|mu', desc='Advance ratio of rotor')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_output('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		v_inf = inputs['v_inf']
		a = inputs['Rotor|alpha']
		r = inputs['Rotor|radius']
		mu = inputs['Rotor|mu']
		
		outputs['Rotor|omega'] = v_inf * np.cos(a) / (mu * r)

	def compute_partials(self, inputs, partials):
		v_inf = inputs['v_inf']
		a = inputs['Rotor|alpha']
		r = inputs['Rotor|radius']
		mu = inputs['Rotor|mu']

		partials['Rotor|omega', 'v_inf'] = np.cos(a) / (mu * r)
		partials['Rotor|omega', 'Rotor|alpha'] = - v_inf * np.sin(a) / (mu * r) 
		partials['Rotor|omega', 'Rotor|mu'] = - v_inf * np.cos(a) / (mu**2 * r)
		partials['Rotor|omega', 'Rotor|radius'] = - v_inf * np.cos(a) / (mu * r**2)

class PropellerRevolutionFromAdvanceRatio(om.ExplicitComponent):
	"""
	Computes the propeller revolution (omega) given the advance ratio
		J = V / (n D)
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|J 		: propeller's advance ratio
		v_inf			: freestream velocity [m/s]
	Outputs:
		Rotor|omega 	: rotor's angular velocity [rad/s]
	"""
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|J', desc='Advance ratio of propeller')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_output('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		v_inf = inputs['v_inf']		# in [m/s]
		r = inputs['Rotor|radius']	# in [m]
		J = inputs['Rotor|J']
		n = v_inf / (2 * r * J) 	# revolutions/second

		outputs['Rotor|omega'] = 2 * np.pi * n # in [rad/s]

	def compute_partials(self, inputs, partials):
		v_inf = inputs['v_inf']		# in [m/s]
		r = inputs['Rotor|radius']	# in [m]
		J = inputs['Rotor|J']

		partials['Rotor|omega', 'v_inf'] = np.pi / (r * J)
		partials['Rotor|omega', 'Rotor|radius'] = - (np.pi * v_inf) / (r**2 * J)
		partials['Rotor|omega', 'Rotor|J'] = - (np.pi * v_inf) / (r * J**2)

class ThrustOfEachRotor(om.ExplicitComponent):
	"""
	Computes the thrust required by each rotor given the weight or drag requirement
	Parameters:
		N_rotor			: Number of rotors
	Inputs:
		Thrust_all		: Thrust required (sum of all rotors) [N]
	Outputs:
		Rotor|Thrust 	: Thrust of each rotor [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')

	def setup(self):
		self.add_input('Thrust_all', units='N', desc='Thrust required (sum of all rotors)')
		self.add_output('Rotor|Thrust', units='N', desc='Thrust required by each rotor')
		# partial is constant
		N_rotor = self.options['N_rotor']
		self.declare_partials('Rotor|Thrust', 'Thrust_all', val=1/N_rotor)

	def compute(self, inputs, outputs):
		outputs['Rotor|Thrust'] = inputs['Thrust_all'] / self.options['N_rotor']
