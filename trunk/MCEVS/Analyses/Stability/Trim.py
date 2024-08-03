import numpy as np
import openmdao.api as om

class MultiRotorTrim(om.ExplicitComponent):
	"""
	Computes the body tilt angle for wingless multirotor in cruise
	Inputs:
		Weight|takeoff 		: total take-off weight [kg]
		Aero|total_drag 	: drag of a multirotor body [N]
	Outputs:
		Thrust 				: total thrust required as a vehicle [N]
		Body|sin_beta		: sin(beta), beta = body incidence angle [rad]
	"""
	def initialize(self):
		self.options.declare('g', default=9.81, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|total_drag', units='N', desc='Drag of a multirotor body')
		self.add_output('Thrust', units='N', desc='Thrust required as a vehicle')
		self.add_output('Body|sin_beta', desc='Sin of body incidence angle')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['Weight|takeoff']	# in [kg]
		D = inputs['Aero|total_drag']					# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff*g)**2 + D**2)

		outputs['Thrust'] = thrust
		outputs['Body|sin_beta'] = (W_takeoff*g) / thrust

	def compute_partials(self, inputs, partials):
		W_takeoff = inputs['Weight|takeoff']	# in [kg]
		D = inputs['Aero|total_drag']					# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff*g)**2 + D**2)

		partials['Thrust', 'Weight|takeoff'] = (W_takeoff * g**2) / thrust
		partials['Thrust', 'Aero|total_drag'] = D / thrust
		partials['Body|sin_beta', 'Weight|takeoff'] = g/thrust - (W_takeoff**2 * g**3)/(thrust**3)
		partials['Body|sin_beta', 'Aero|total_drag'] = - (W_takeoff*g*D) / (thrust**3)
