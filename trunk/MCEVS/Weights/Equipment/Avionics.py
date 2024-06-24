import numpy as np
import openmdao.api as om

class AvionicsWeight(om.ExplicitComponent):
	"""
	Computes avionics system weight
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
	Outputs:
		Weights|Avionics : weight of all avionics systems [kg]
	Notes:
		> This equation has not been checked against old-established literatures
	Source:
		1. Kadhiresan, A. R., and Duffy, M. J., “Conceptual Design and Mission Analysis for EVTOL Urban Air Mobility Flight Vehicle Configurations,”
		   presented at the AIAA Aviation 2019 Forum, Dallas, Texas, 2019. https://doi.org/10.2514/6.2019-2873
  	"""
	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Weights|Avionics', units='kg', desc='Weight of all avionics systems')
		self.declare_partials('Weights|Avionics', 'eVTOL|W_takeoff')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['eVTOL|W_takeoff'] # in [kg]

		# Calculating W_avionics
		W_avionics = 0.0268 * W_takeoff

		outputs['Weights|Avionics'] = W_avionics # in [kg]

	def compute_partials(self, inputs, partials):
		partials['Weights|Avionics', 'eVTOL|W_takeoff'] = 0.0268


