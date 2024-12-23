import numpy as np
import openmdao.api as om

class PowerHoverClimbConstantSpeed(om.ExplicitComponent):
	"""
	Computes the power required for hover climb with constant speed
	Parameters:
		N_rotor		 : number or lift rotors
		hover_FM	 : hover figure of merit
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
		v_climb 	 : climb speed [m/s]
	Inputs:
		Weight|takeoff  	: total take-off weight [kg]
		LiftRotor|radius	: lift rotor radius [m]
	Outputs:
		Power|HoverClimbConstantSpeed	: power required for hover climb [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover climb [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('v_climb', types=float, desc='Hover climb speed')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_output('Power|HoverClimbConstantSpeed', units='W', desc='Power required for hover climb')
		self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']
		v_climb = self.options['v_climb']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		outputs['Power|HoverClimbConstantSpeed'] = (W_takeoff*g)/hover_FM * ( (v_climb/2) + np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		outputs['LiftRotor|thrust'] = (W_takeoff*g)/N_rotor

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']
		v_climb = self.options['v_climb']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		partials['Power|HoverClimbConstantSpeed', 'Weight|takeoff'] = g*v_climb/(2*hover_FM) + g/hover_FM * np.sqrt( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) + (W_takeoff*g**2)/(4*hover_FM*rho_air*S_disk*N_rotor) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5)
		partials['Power|HoverClimbConstantSpeed', 'LiftRotor|radius'] = - (W_takeoff*g)**2 / (4*hover_FM*rho_air*N_rotor*S_disk**2) * ( (v_climb/2)**2 + (W_takeoff*g)/(2*rho_air*N_rotor*S_disk) )**(-0.5) * dSdisk_dr
		partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor

