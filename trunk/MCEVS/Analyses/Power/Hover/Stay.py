import numpy as np
import openmdao.api as om

class PowerHoverStay(om.ExplicitComponent):
	"""
	Computes the power required for hover stay (staying still in the air)
	Parameters:
		N_rotor		 : number or lift rotors
		hover_FM	 : hover figure of merit
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
	Inputs:
		Weight|takeoff  	: total take-off weight [kg]
		LiftRotor|radius	: lift rotor radius [m]
	Outputs:
		Power|HoverStay	: power required for hover stay [W]
		LiftRotor|thrust : thrust produced by each rotor during hover [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', default=1.225, desc='Air density')
		self.options.declare('g', default=9.81, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_output('Power|HoverStay', units='W', desc='Power required for hover stay')
		self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		outputs['Power|HoverStay'] = 1/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*S_disk*N_rotor))
		outputs['LiftRotor|thrust'] = (W_takeoff*g)/N_rotor

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		partials['Power|HoverStay', 'Weight|takeoff'] = 1.5/hover_FM * np.sqrt((W_takeoff * g**3)/(2*rho_air*S_disk*N_rotor))
		partials['Power|HoverStay', 'LiftRotor|radius'] = -0.5/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*N_rotor*S_disk**3)) * dSdisk_dr
		partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor

