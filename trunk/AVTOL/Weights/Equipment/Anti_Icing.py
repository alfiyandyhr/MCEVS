import numpy as np
import openmdao.api as om

class AntiIcingSystemWeight(om.ExplicitComponent):
	"""
	Computes anti-icing system weight
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
	Outputs:
		eVTOL|W_anti_icing : weight of all anti-icing systems [kg]
	Notes:
		> It also takes into account air conditioning system
	Source:
		Prouty, R. W., Helicopter Performance, Stability, and Control, Krieger, 2002.
	"""
	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_output('eVTOL|W_anti_icing', units='kg', desc='Weight of all anti-icing systems')
		self.declare_partials('eVTOL|W_anti_icing', 'eVTOL|W_takeoff')

	def compute(self, inputs, outputs):
		W_takeoff = inputs['eVTOL|W_takeoff'] # in [kg]

		# Calculating W_anti_icing
		W_anti_icing = 8.0 * (W_takeoff/1000.0)

		outputs['eVTOL|W_anti_icing'] = W_anti_icing # in [kg]

	def compute_partials(self, inputs, partials):
		partials['eVTOL|W_anti_icing', 'eVTOL|W_takeoff'] = 0.008


