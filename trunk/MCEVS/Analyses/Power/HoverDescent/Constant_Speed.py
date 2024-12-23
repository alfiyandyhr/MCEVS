import numpy as np
import openmdao.api as om

class PowerHoverDescentConstantSpeed(om.ExplicitComponent):
	"""
	Computes the power required for hover descent with constant speed
	Parameters:
		N_rotor		 : number or lift rotors
		hover_FM	 : hover figure of merit
		rho_air		 : air density [kg/m**3]
		g 			 : gravitational acceleration [m/s**2]
		v_descent 	 : descent speed [m/s]
	Inputs:
		Weight|takeoff  	: total take-off weight [kg]
		LiftRotor|radius	: lift rotor radius [m]
	Outputs:
		Power|HoverDescentConstantSpeed	: power required for hover descent [W]
		LiftRotor|thrust 				: thrust produced by each rotor during hover descent [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('g', types=float, desc='Gravitational acceleration')
		self.options.declare('v_descent', types=float, desc='Hover descent speed')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('LiftRotor|radius', units='m', desc='Lift rotor radius')
		self.add_output('Power|HoverDescentConstantSpeed', units='W', desc='Power required for hover descent')
		self.add_output('LiftRotor|thrust', units='N', desc='Thrust of each rotor during hover')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']
		v_descent = self.options['v_descent']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2

		# If v_descent is less than 2*v_hoverstay, use original hover descent model, otherwise use hover stay model
		if v_descent >= 2.0 * np.sqrt((W_takeoff*g)/(2*rho_air*S_disk*N_rotor)):
			outputs['Power|HoverDescentConstantSpeed'] = (W_takeoff*g)/hover_FM * ( (v_descent/2) - np.sqrt( (v_descent/2)**2 - (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) )
		else:
			outputs['Power|HoverDescentConstantSpeed'] = 1/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*S_disk*N_rotor))
		outputs['LiftRotor|thrust'] = (W_takeoff*g)/N_rotor

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']
		v_descent = self.options['v_descent']

		W_takeoff = inputs['Weight|takeoff']
		r = inputs['LiftRotor|radius'] # in [m]

		S_disk = np.pi * r**2
		dSdisk_dr = 2 * np.pi * r

		# If v_descent is less than 2*v_hoverstay, use original hover descent model, otherwise use hover stay model
		if v_descent >= 2.0 * np.sqrt((W_takeoff*g)/(2*rho_air*S_disk*N_rotor)):
			partials['Power|HoverDescentConstantSpeed', 'Weight|takeoff'] = g*v_descent/(2*hover_FM) - g/hover_FM * np.sqrt( (v_descent/2)**2 - (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) ) + (W_takeoff*g**2)/(4*hover_FM*rho_air*S_disk*N_rotor) * ( (v_descent/2)**2 - (W_takeoff*g)/(2*rho_air*S_disk*N_rotor) )**(-0.5)
			partials['Power|HoverDescentConstantSpeed', 'LiftRotor|radius'] = - (W_takeoff*g)**2 / (4*hover_FM*rho_air*N_rotor*S_disk**2) * ( (v_descent/2)**2 - (W_takeoff*g)/(2*rho_air*N_rotor*S_disk) )**(-0.5) * dSdisk_dr
		else:
			partials['Power|HoverDescentConstantSpeed', 'Weight|takeoff'] = 1.5/hover_FM * np.sqrt((W_takeoff * g**3)/(2*rho_air*S_disk*N_rotor))
			partials['Power|HoverDescentConstantSpeed', 'LiftRotor|radius'] = - 0.5/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*N_rotor*S_disk**3)) * dSdisk_dr
		partials['LiftRotor|thrust', 'Weight|takeoff'] = g / N_rotor

