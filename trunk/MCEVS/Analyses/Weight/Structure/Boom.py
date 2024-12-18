import numpy as np
import openmdao.api as om

class BoomWeight(om.ExplicitComponent):
	"""
	Computes fuselage weight
	Parameters:
		tf 				: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		total_req_takeoff_power : total power required for liftrotors during hover climb (adapted for eVTOL) [W]
	Outputs:
		Weight|boom 	: boom(s) weight [kg]
	Notes:
		> Torenbeek Method
		> Used for multi-engine airplane with piston (turboprop) engines
		> Since it uses total_req_takeoff_power, it estimates the weights of all nacelles (booms)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('total_req_takeoff_power', units='W', desc='Total required takeoff power')
		self.add_output('Weight|boom', units='kg', desc='Boom weight')
		self.declare_partials('Weight|boom', 'total_req_takeoff_power')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		total_req_takeoff_power = inputs['total_req_takeoff_power']	# in [W]

		# Calculating W_boom
		W_to_hp = 1/745.7
		lb_to_kg = 0.453592
		
		W_boom = 0.14 * total_req_takeoff_power * W_to_hp * lb_to_kg

		outputs['Weight|boom'] = tf * W_boom # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		total_req_takeoff_power = inputs['total_req_takeoff_power']	# in [W]

		W_to_hp = 1/745.7
		lb_to_kg = 0.453592
		dWboom_dPtakeoff = 0.14 * W_to_hp * lb_to_kg

		partials['Weight|boom', 'total_req_takeoff_power'] = tf * dWboom_dPtakeoff
