import numpy as np
import openmdao.api as om

class MultiRotorTrim(om.ExplicitComponent):
	"""
	Computes the body tilt angle for wingless multirotor in cruise
	Inputs:
		eVTOL|W_takeoff 	: total take-off weight [kg]
		Aero|Drag 			: drag of a multirotor body [N]
	Outputs:
		Thrust 				: total thrust required as a vehicle [N]
		Body|sin_beta		: sin(beta), beta = body incidence angle [rad]
	"""
	def initialize(self):
		self.options.declare('g', default=9.81, desc='Gravitational acceleration')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Aero|Drag', units='N', desc='Drag of a multirotor body')
		self.add_output('Thrust', units='N', desc='Thrust required as a vehicle')
		self.add_output('Body|sin_beta', desc='Sin of body incidence angle')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		D = inputs['Aero|Drag']					# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff*g)**2 + D**2)

		outputs['Thrust'] = thrust
		outputs['Body|sin_beta'] = (W_takeoff*g) / thrust

	def compute_partials(self, inputs, partials):
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		D = inputs['Aero|Drag']					# in [N]
		g = self.options['g']					# in [m/s**2]
		thrust = np.sqrt((W_takeoff*g)**2 + D**2)

		partials['Thrust', 'eVTOL|W_takeoff'] = (W_takeoff * g**2) / thrust
		partials['Thrust', 'Aero|Drag'] = D / thrust
		partials['Body|sin_beta', 'eVTOL|W_takeoff'] = g/thrust - (W_takeoff**2 * g**3)/(thrust**3)
		partials['Body|sin_beta', 'Aero|Drag'] = - (W_takeoff*g*D) / (thrust**3)
