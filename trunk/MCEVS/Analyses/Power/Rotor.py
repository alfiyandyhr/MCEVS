import numpy as np
import openmdao.api as om
import warnings

from MCEVS.Utils.Functions import SoftMax

class PowerForwardComp(om.ExplicitComponent):
	"""
	Computes power required for forward flight
	Parameters:
		N_rotor	: number or rotors
		g 		: gravitational acceleration
	Inputs:
		Rotor|thrust		: thrust of a rotor [N]
		Rotor|profile_power	: profile power of a rotor [W]
		Rotor|alpha			: rotor tilt angle [rad]
		Rotor|kappa			: induced power factor
		v_inf 				: freestream velocity [m/s]
		v_induced 			: induced velocity by a rotor [m/s]
	Outputs:
		Power|forward 		: required power for forward flight [W]
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
		self.options.declare('g', types=float, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('Rotor|thrust', units='N', desc='Thrust of each rotor')
		self.add_input('Rotor|profile_power', units='W', desc='Profile power of each rotor, P0')
		self.add_input('Rotor|alpha', units='rad', desc='Rotor tilt angle: 90 for being a propeller, 0 for hover')
		self.add_input('Rotor|kappa', desc='Induced power factor')
		self.add_input('v_inf', units='m/s', desc='Freestream velocity')
		self.add_input('v_induced', units='m/s', desc='Induced velocity')
		self.add_output('Power|forward', units='W', desc='Power required for forward flight (sum of all rotors)')
		self.add_output('Power|profile_power', units='W', desc='Profile power (sum of all rotors)')
		self.add_output('Power|induced_power', units='W', desc='Induced power (sum of all rotors)')
		self.add_output('Power|propulsive_power', units='W', desc='Propulsive power (sum of all rotors)')
		self.add_output('Rotor|T_to_P', units='g/W', desc='Thrust to power ratio of a single rotor/propeller')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		g = self.options['g']
		P0 = inputs['Rotor|profile_power']
		T_rotor = inputs['Rotor|thrust']
		a = inputs['Rotor|alpha']
		k = inputs['Rotor|kappa']
		v_inf = inputs['v_inf']
		v_ind = inputs['v_induced']

		power_fwd_each = P0 + T_rotor * (k*v_ind + v_inf*np.sin(a))

		outputs['Power|forward'] = N_rotor * power_fwd_each
		outputs['Power|profile_power'] = N_rotor * P0
		outputs['Power|induced_power'] = N_rotor * T_rotor * k * v_ind
		outputs['Power|propulsive_power'] = N_rotor * T_rotor * v_inf*np.sin(a)
		outputs['Rotor|T_to_P'] = (T_rotor / g * 1000.0) / power_fwd_each
		# print(N_rotor, g, P0, T_rotor, a, k, v_inf, v_ind, outputs['Rotor|T_to_P'])

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		g = self.options['g']
		P0 = inputs['Rotor|profile_power']
		T_rotor = inputs['Rotor|thrust']
		a = inputs['Rotor|alpha']
		k = inputs['Rotor|kappa']
		v_inf = inputs['v_inf']
		v_ind = inputs['v_induced']

		power_fwd_each = P0 + T_rotor * (k*v_ind + v_inf*np.sin(a))
		dP_dT = k*v_ind + v_inf*np.sin(a)
		dP_dP0 = 1.0
		dP_da = T_rotor * v_inf*np.cos(a)
		dP_dk = T_rotor * v_ind
		dP_dv1 = T_rotor * np.sin(a)
		dP_dv2 = T_rotor * k

		partials['Power|forward', 'Rotor|thrust'] = N_rotor * dP_dT
		partials['Power|forward', 'Rotor|profile_power'] = N_rotor * dP_dP0
		partials['Power|forward', 'Rotor|alpha'] = N_rotor * dP_da
		partials['Power|forward', 'Rotor|kappa'] = N_rotor * dP_dk
		partials['Power|forward', 'v_inf'] = N_rotor * dP_dv1
		partials['Power|forward', 'v_induced'] = N_rotor * dP_dv2

		partials['Power|profile_power', 'Rotor|thrust'] = 0.0
		partials['Power|profile_power', 'Rotor|profile_power'] = N_rotor
		partials['Power|profile_power', 'Rotor|alpha'] = 0.0
		partials['Power|profile_power', 'Rotor|kappa'] = 0.0
		partials['Power|profile_power', 'v_inf'] = 0.0
		partials['Power|profile_power', 'v_induced'] = 0.0

		partials['Power|induced_power', 'Rotor|thrust'] = N_rotor * k * v_ind
		partials['Power|induced_power', 'Rotor|profile_power'] = 0.0
		partials['Power|induced_power', 'Rotor|alpha'] = 0.0
		partials['Power|induced_power', 'Rotor|kappa'] = N_rotor * T_rotor * v_ind
		partials['Power|induced_power', 'v_inf'] = 0.0
		partials['Power|induced_power', 'v_induced'] = N_rotor * T_rotor * k

		partials['Power|propulsive_power', 'Rotor|thrust'] = N_rotor * v_inf*np.sin(a)
		partials['Power|propulsive_power', 'Rotor|profile_power'] = 0.0
		partials['Power|propulsive_power', 'Rotor|alpha'] = N_rotor * T_rotor * v_inf*np.cos(a)
		partials['Power|propulsive_power', 'Rotor|kappa'] = 0.0
		partials['Power|propulsive_power', 'v_inf'] = N_rotor * T_rotor * np.sin(a)
		partials['Power|propulsive_power', 'v_induced'] = 0.0

		partials['Rotor|T_to_P', 'Rotor|thrust'] = (1 / g * 1000.0) / power_fwd_each + (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_dT
		partials['Rotor|T_to_P', 'Rotor|profile_power'] = (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_dP0
		partials['Rotor|T_to_P', 'Rotor|alpha'] = (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_da
		partials['Rotor|T_to_P', 'Rotor|kappa'] = (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_dk
		partials['Rotor|T_to_P', 'v_inf'] = (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_dv1
		partials['Rotor|T_to_P', 'v_induced'] = (T_rotor / g * 1000.0) * (-1/power_fwd_each**2) * dP_dv2

class RotorProfilePower(om.ExplicitComponent):
	"""
	Computes the profile power of a rotor
	Parameters:
		n_blade 		: number of blades per rotor
		rho_air			: air density [kg/m**3]
		Cd0 			: zero lift drag of a rotor
	Inputs:
		Rotor|radius	: rotor radius [m]
		Rotor|chord 	: rotor chord length [m]
		Rotor|mu 		: rotor's advance ratio
		Rotor|omega 	: rotor's angular velocity [rad/s]
	Outputs:
		Rotor|profile_power	: profile power of a rotor [W]
	Source:
		Johnson, W., “Rotorcraft Aeromechanics,” Cambridge University Press, 2013.
	"""
	def initialize(self):
		self.options.declare('n_blade', desc='Number of blades per rotor')
		self.options.declare('rho_air', default=1.225, desc='Air density')
		self.options.declare('Cd0', default=0.012, desc='Zero lift drag of a rotor')

	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|chord', units='m', desc='Rotor chord length')
		self.add_input('Rotor|mu', desc='Rotor advance ratio')
		self.add_input('Rotor|omega', units='rad/s', desc='Rotor angular velocity')
		self.add_output('Rotor|profile_power', units='W', desc='Profile power of a rotor, P0')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		n_blade = self.options['n_blade']
		rho_air = self.options['rho_air']
		Cd0 = self.options['Cd0']
		mu = inputs['Rotor|mu']
		omega = inputs['Rotor|omega']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']
		sigma = n_blade * c / (np.pi * r)
		
		P0_each = (sigma*Cd0/8) * (1 + 4.65*mu**2) * (np.pi * rho_air * omega**3 * r**5)
		outputs['Rotor|profile_power'] = P0_each

		# print(n_blade, rho_air, Cd0, mu, omega, sigma, P0_each)

	def compute_partials(self, inputs, partials):
		n_blade = self.options['n_blade']
		rho_air = self.options['rho_air']
		Cd0 = self.options['Cd0']
		mu = inputs['Rotor|mu']
		omega = inputs['Rotor|omega']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']
		sigma = n_blade * c / (np.pi * r)

		k1 = sigma * Cd0 / 8
		k2 = 1 + 4.65*mu**2
		k3 = np.pi * rho_air * omega**3 * r**5

		partials['Rotor|profile_power', 'Rotor|mu'] = k1 * k3 * (2 * 4.65 * mu)
		partials['Rotor|profile_power', 'Rotor|omega'] = k1 * k2 * (np.pi * rho_air * 3 * omega**2 * r**5)
		partials['Rotor|profile_power', 'Rotor|radius'] = n_blade * c / np.pi * Cd0 / 8 * k2 * (np.pi * rho_air * omega**3 * 4 * r**3)
		partials['Rotor|profile_power', 'Rotor|chord'] = n_blade / np.pi * Cd0 / 8 * k2 * (np.pi * rho_air * omega**3 * r**4)

class InducedPowerFactorComp(om.ExplicitComponent):
	"""
	Computes the induced power factor kappa in forward flight
	Inputs:
		Rotor|thrust 		: thrust of a rotor [N]
		Rotor|profile_power	: profile power of a rotor [W]
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
		self.add_input('Rotor|thrust', units='N', desc='Thrust of a rotor')
		self.add_input('Rotor|profile_power', units='W', desc='Profile power of a rotor, P0')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_output('kappa_raw', desc='Induced power factor')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|thrust']
		P0_each = inputs['Rotor|profile_power']
		r = inputs['Rotor|radius']

		S_disk = np.pi * r**2

		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			# sometimes warnings pop up due to negative thrust
			kappa = 1/hover_FM -  P0_each * np.sqrt((2*rho_air*S_disk)/thrust**3)

		outputs['kappa_raw'] = kappa

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		thrust = inputs['Rotor|thrust']
		P0_each = inputs['Rotor|profile_power']
		r = inputs['Rotor|radius']

		S_disk = np.pi * r**2
		dk_dt = 1.5 * P0_each * np.sqrt((2*rho_air*S_disk)/thrust**5)
		dk_dp = - np.sqrt((2*rho_air*S_disk)/thrust**3)
		dk_dr = - P0_each * np.sqrt(rho_air/(2*S_disk*thrust**3)) * 2*np.pi*r

		partials['kappa_raw', 'Rotor|thrust'] = dk_dt
		partials['kappa_raw', 'Rotor|profile_power'] = dk_dp 
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
