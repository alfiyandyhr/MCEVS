import numpy as np
import openmdao.api as om

from MCEVS.Utils.Functions import SoftMax

class PowerForwardComp(om.ExplicitComponent):
	"""
	Computes power required for forward flight (cruising segment)
	Parameters:
		N_rotor	: number or rotors
	Inputs:
		Rotor|Thrust		: thrust of a rotor [N]
		Rotor|Profile_power	: profile power of a rotor [W]
		Rotor|alpha			: rotor tilt angle [rad]
		Rotor|kappa			: induced power factor
		v_inf 				: freestream velocity [m/s]
		v_induced 			: induced velocity by a rotor [m/s]
	Outputs:
		power_forward 		: required power for cruise [W]
	Notes:
		> Composed of three components:
			1. Profile power
				Power intrinsic to overcome the drag due to the rotation of its blades
			2. Induced power
				Power to propel the air and convert it into thrust
			3. Parasite power
				Power required to overcome the overall drag
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')

	def setup(self):
		self.add_input('Rotor|Thrust', units='N', desc='Thrust of each rotor')
		self.add_input('Rotor|Profile_power', units='W', desc='Profile power of each rotor, P0')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle: 90 for being a propeller, 0 for hover')
		self.add_input('Rotor|kappa', desc='Induced power factor')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_input('v_induced', units='m/s', desc='Induced velocity')
		self.add_output('power_forward', units='W', desc='Power required for forward flight (sum of all rotors)')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		P0 = inputs['Rotor|Profile_power']
		T_rotor = inputs['Rotor|Thrust']
		a = inputs['Rotor|alpha']
		k = inputs['Rotor|kappa']
		v_inf = inputs['v_inf']
		v_ind = inputs['v_induced']

		power_fwd_each = P0 + T_rotor * (k*v_ind + v_inf*np.sin(a))
		outputs['power_forward'] = N_rotor * power_fwd_each

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		P0 = inputs['Rotor|Profile_power']
		T_rotor = inputs['Rotor|Thrust']
		a = inputs['Rotor|alpha']
		k = inputs['Rotor|kappa']
		v_inf = inputs['v_inf']
		v_ind = inputs['v_induced']

		partials['power_forward', 'Rotor|Thrust'] = N_rotor * (k*v_ind + v_inf*np.sin(a))
		partials['power_forward', 'Rotor|Profile_power'] = N_rotor
		partials['power_forward', 'Rotor|alpha'] = N_rotor * T_rotor * v_inf*np.cos(a)
		partials['power_forward', 'Rotor|kappa'] = N_rotor * T_rotor * v_ind
		partials['power_forward', 'v_inf'] = N_rotor * T_rotor * np.sin(a)
		partials['power_forward', 'v_induced'] = N_rotor * T_rotor * k

class RotorProfilePower(om.ExplicitComponent):
	"""
	Computes the profile power of a rotor
	Parameters:
		rho_air			: air density [kg/m**3]
		rotor_sigma 	: rotor's solidity
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|mu 		: rotor's advance ratio
		Rotor|omega 	: rotor's angular velocity [rad/s]
	Outputs:
		Rotor|Profile_power	: profile power of a rotor [W]
	Source:
		Johnson, W., “Rotorcraft Aeromechanics,” Cambridge University Press, 2013.
	"""
	def initialize(self):
		self.options.declare('rho_air', default=1.225, desc='Air density')
		self.options.declare('sigma', types=float, default=0.13, desc='Rotor solidity, e.g., 0.13')
		self.options.declare('Cd0', default=0.012, desc='Zero lift drag of a rotor')

	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|mu', desc='Rotor advance ratio')
		self.add_input('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.add_output('Rotor|Profile_power', units='W', desc='Profile power of a rotor, P0')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		rho_air = self.options['rho_air']
		sigma = self.options['sigma']
		Cd0 = self.options['Cd0']
		mu = inputs['Rotor|mu']
		omega = inputs['Rotor|omega']
		r = inputs['Rotor|radius']

		P0_each = (sigma*Cd0/8) * (1 + 4.65*mu**2) * (np.pi * rho_air * omega**3 * r**5)
		outputs['Rotor|Profile_power'] = P0_each

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		sigma = self.options['sigma']
		Cd0 = self.options['Cd0']
		mu = inputs['Rotor|mu']
		omega = inputs['Rotor|omega']
		r = inputs['Rotor|radius']

		k1 = sigma * Cd0 / 8
		k2 = 1 + 4.65*mu**2
		k3 = np.pi * rho_air * omega**3 * r**5

		partials['Rotor|Profile_power', 'Rotor|mu'] = k1 * k3 * (2 * 4.65 * mu)
		partials['Rotor|Profile_power', 'Rotor|omega'] = k1 * k2 * (np.pi * rho_air * 3 * omega**2 * r**5)
		partials['Rotor|Profile_power', 'Rotor|radius'] = k1 * k2 * (np.pi * rho_air * omega**3 * 5 * r**4)

class InducedPowerFactorComp(om.ExplicitComponent):
	"""
	Computes the induced power factor kappa in forward flight
	Inputs:
		Rotor|Thrust 		: thrust of a rotor [N]
		Rotor|Profile_power	: profile power of a rotor [W]
		Rotor|radius		: rotor radius [m]
	Outputs:
		kappa_raw	: induced power factor before being softmax-ed
	Source:
		B. Govindarajan and A. Sridharan, “Conceptual Sizing of Vertical Lift Package Delivery Platforms,”
		Journal of Aircraft, vol. 57, no. 6, pp. 1170–1188, Nov. 2020, doi: 10.2514/1.C035805.
	"""
	def initialize(self):
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')	
		self.options.declare('rho_air', types=float, default=1.225, desc='Air density')

	def setup(self):
		self.add_input('Rotor|Thrust', units='N', desc='Thrust of a rotor')
		self.add_input('Rotor|Profile_power', units='W', desc='Profile power of a rotor, P0')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_output('kappa_raw', desc='Induced power factor')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|Thrust']
		P0_each = inputs['Rotor|Profile_power']
		r = inputs['Rotor|radius']

		S_disk = np.pi * r**2
		kappa = 1/hover_FM -  P0_each * np.sqrt((2*rho_air*S_disk)/thrust**3)
		outputs['kappa_raw'] = kappa

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|Thrust']
		P0_each = inputs['Rotor|Profile_power']
		r = inputs['Rotor|radius']

		S_disk = np.pi * r**2
		dk_dt = 1.5 * P0_each * np.sqrt((2*rho_air*S_disk)/thrust**5)
		dk_dp = - np.sqrt((2*rho_air*S_disk)/thrust**3)
		dk_dr = - P0_each * np.sqrt(rho_air/(2*S_disk*thrust**3)) * 2*np.pi*r

		partials['kappa_raw', 'Rotor|Thrust'] = dk_dt
		partials['kappa_raw', 'Rotor|Profile_power'] = dk_dp 
		partials['kappa_raw', 'Rotor|radius'] = dk_dr

class InducedPowerFactor(om.Group):
	"""
	Apply softmax to the induced power factor
	Inputs:
		*
	Outputs:
		Rotor|kappa : induced power factor after being softmaxed
	Source:
		B. Govindarajan and A. Sridharan, “Conceptual Sizing of Vertical Lift Package Delivery Platforms,”
		Journal of Aircraft, vol. 57, no. 6, pp. 1170–1188, Nov. 2020, doi: 10.2514/1.C035805.
	"""
	def initialize(self):
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', default=1.225, desc='Air density')

	def setup(self):
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']

		# Compute kappa value
		self.add_subsystem('kappa_raw',
							InducedPowerFactorComp(hover_FM=hover_FM, rho_air=rho_air),
							promotes_inputs=['*'])
		# minimum value of kappa
		indep = self.add_subsystem('kappa_min', om.IndepVarComp())
		indep.add_output('kappa_min', val=1.15)

		# kappa = SoftMax(kappa_raw, kappa_min)
		self.add_subsystem('softmax',
							SoftMax(rho=30),
							promotes_outputs=[('fmax', 'Rotor|kappa')])

		self.connect('kappa_raw.kappa_raw', 'softmax.f1')
		self.connect('kappa_min.kappa_min', 'softmax.f2')
