import numpy as np
import openmdao.api as om

class ThrustCoefficient(om.ExplicitComponent):
	"""
	Computes the thrust coefficient of a rotor
	Parameter:
		rho_air 		: air density [kg/m**3]
	Inputs:
		Rotor|Thrust 	: thrust of a rotor [N]
		Rotor|radius	: rotor radius [m]
		Rotor|omega 	: rotor's angular velocity [rad/s]
	Outputs:
		Rotor|Ct 		: rotor's thrust coefficient
	Source:
		Leishman, J. G., Principles of Helicopter Aerodynamics, 2nd ed., Cambridge Aerospace Series, Cambridge University Press, Cambridge, England, UK, 2006.
	"""
	def initialize(self):
		self.options.declare('rho_air', default=1.225, units='kg/m**3', desc='Air density')

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
		partials['Rotor|Ct', 'UAV|r_rotor'] = thrust/(np.pi*rho_air*omega**2) * (-4/r**5)
