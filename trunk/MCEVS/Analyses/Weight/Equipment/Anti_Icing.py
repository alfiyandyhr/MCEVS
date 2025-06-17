import numpy as np
import openmdao.api as om

class AntiIcingSystemWeight(om.ExplicitComponent):
	"""
	Computes anti-icing system weight
	Parameters:
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff : total take-off weight [kg]
	Outputs:
		Weight|equipment|anti_icing : weight of all anti-icing systems [kg]
	Notes:
		> It also takes into account air conditioning system
	Source:
		Prouty, R. W., Helicopter Performance, Stability, and Control, Krieger, 2002.
	"""
	def initialize(self):
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Weight|equipment|anti_icing', units='kg', desc='Weight of all anti-icing systems')
		self.declare_partials('Weight|equipment|anti_icing', 'Weight|takeoff')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		W_takeoff = inputs['Weight|takeoff'] # in [kg]

		# Calculating W_anti_icing
		W_anti_icing = 8.0 * (W_takeoff/1000.0)

		outputs['Weight|equipment|anti_icing'] = tf * W_anti_icing # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		partials['Weight|equipment|anti_icing', 'Weight|takeoff'] = tf * 0.008


