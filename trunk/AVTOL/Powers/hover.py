import numpy as np
import openmdao.api as om

class PowerHover(om.ExplicitComponent):
	"""
	Computes the power required for hover
	Parameters:
		N_rotor		: number or lifting rotors
		hover_FM	: hover figure of merit
		rho_air		: air density [kg/m**3]
		g 			: gravitational acceleration [m/s**2]
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
		Rotor|Diameter	: lifting rotor diameter [m]
	Outputs:
		power_hover	: power required for hover [W]
		thrust_each : thrust produced by each rotor during hover [N]
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
		self.options.declare('rho_air', default=1.225, desc='Air density')
		self.options.declare('g', default=9.81, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Rotor|Diameter', units='m', desc='Lifting rotor diameter')
		self.add_output('power_hover', units='W', desc='Power required for hover')
		self.add_output('thrust_each', units='N', desc='Thrust of each rotor during hover')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['eVTOL|W_takeoff']
		D = inputs['Rotor|Diameter'] # in [m]

		S_disk = 0.25 * np.pi * D**2
		outputs['power_hover'] = 1/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*S_disk*N_rotor))
		outputs['thrust_each'] = (W_takeoff*g)/N_rotor

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		hover_FM = self.options['hover_FM']
		rho_air = self.options['rho_air']
		g = self.options['g']

		W_takeoff = inputs['eVTOL|W_takeoff']
		D = inputs['Rotor|Diameter'] # in [m]

		S_disk = 0.25 * np.pi * D**2
		dSdisk_dD = 0.5 * np.pi * D

		partials['power_hover', 'eVTOL|W_takeoff'] = 1.5/hover_FM * np.sqrt((W_takeoff * g**3)/(2*rho_air*S_disk*N_rotor))
		partials['power_hover', 'Rotor|Diameter'] = -0.5/hover_FM * np.sqrt(((W_takeoff*g)**3)/(2*rho_air*N_rotor*S_disk**3)) * dSdisk_dD
		partials['thrust_each', 'eVTOL|W_takeoff'] = g / N_rotor