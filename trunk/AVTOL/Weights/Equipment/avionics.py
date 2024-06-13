import numpy as np
import openmdao.api as om

class AvionicsWeight(om.ExplicitComponent):
	"""
	Computes avionics system weight
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
	Outputs:
		eVTOL|W_avionics : weight of all avionics systems [kg]
	Notes:
		> This equation has not been checked against old-established literatures
	Source:
		Kadhiresan 2019, Conceptual Design and Mission Analysis for eVTOL Urban Air Mobility Flight Vehicle Configurations
	"""
	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_output('eVTOL|W_avionics', units='kg', desc='Weight of all avionics systems')
		self.declare_partials('eVTOL|W_avionics', 'eVTOL|W_takeoff')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['eVTOL|W_takeoff'] # in [kg]

		# Calculating W_avionics
		W_avionics = 0.0268 * W_takeoff

		outputs['eVTOL|W_avionics'] = W_avionics # in [kg]

	def compute_partials(self, inputs, partials):
		partials['eVTOL|W_avionics', 'eVTOL|W_takeoff'] = 0.0268


