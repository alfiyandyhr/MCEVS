import numpy as np
import openmdao.api as om

class MotorWeight(om.ExplicitComponent):
	"""
	Computes motor weight
	Parameters:
		N_motor	: number of motors
	Inputs:
		max_power : maximum power, i.e., power during climb [W]
	Outputs:
		Weights|Motors : weight of all motors [kg]
	Notes:
		> An empirical equation based on power density regression on DC electric motors
		> It is suggested to renew the equation as more data become available
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weights|Motors', units='kg', desc='Weight of all motors')
		self.declare_partials('Weights|Motors', 'max_power')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		p_max = inputs['max_power']/1000.0 # in [kW]

		# Calculating W_motors
		W_motor = (0.188 * p_max + 5.836)/N_motor
		W_motors = N_motor * W_motor

		outputs['Weights|Motors'] = W_motors # in [kg]

	def compute_partials(self, inputs, partials):
		partials['Weights|Motors', 'max_power'] = 0.188





