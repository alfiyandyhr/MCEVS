import numpy as np
import openmdao.api as om

class PowerConstantFractionOfMaxPower(om.ExplicitComponent):
	"""
	Computes the power required for ConstantPowerSegment
	Parameters:
		percent_max_power
	Inputs:
		max_power
	Outputs:
		fractional_power
		zero_thrust
	"""
	def initialize(self):
		self.options.declare('percent_max_power', types=float, desc='Percentage of power from max power')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum deliverable power')
		self.add_output('fractional_power', units='W', desc='Fractional power')
		self.add_output('zero_thrust', units='N', desc='Zero thrust assumption')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		percent_max_power = self.options['percent_max_power']/100
		max_power = inputs['max_power']

		outputs['fractional_power'] = percent_max_power * max_power
		outputs['zero_thrust'] = 0.0

	def compute_partials(self, inputs, partials):
		percent_max_power = self.options['percent_max_power']/100

		partials['fractional_power','max_power'] = percent_max_power
		partials['zero_thrust','max_power'] = 0.0

