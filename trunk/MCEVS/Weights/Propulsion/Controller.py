import numpy as np
import openmdao.api as om

class MotorControllerWeight(om.ExplicitComponent):
	"""
	Computes motor controller weight
	Parameters:
		N_motor	: number of motors
	Inputs:
		max_power : maximum power, i.e., power during climb [W]
	Outputs:
		Weights|MotorControllers : weight of all motor controllers [kg]
	Notes:
	Sources:
		1. Kadhiresan, A. R., and Duffy, M. J., “Conceptual Design and Mission Analysis for EVTOL Urban Air Mobility Flight Vehicle Configurations,”
		   presented at the AIAA Aviation 2019 Forum, Dallas, Texas, 2019. https://doi.org/10.2514/6.2019-2873
  		2. Duffy, M., Sevier, A. E., Hupp, R., Perdomo, E., and Wakayama, S., “Propulsion Scaling Methods in the Era of Electric Flight,”
  		   presented at the 2018 AIAA/IEEE Electric Aircraft Technologies Symposium, Cincinnati, Ohio, 2018. https://doi.org/10.2514/6.2018-4978
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weights|MotorControllers', units='kg', desc='Weight of all motor controllers')
		self.declare_partials('Weights|MotorControllers', 'max_power')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		p_max = inputs['max_power']/1000.0 # in [kW]

		# Calculating W_controllers
		W_controller = 49.9/398 * (p_max/N_motor - 2.0) + 0.1
		W_controllers = N_motor * W_controller

		outputs['Weights|MotorControllers'] = W_controllers # in [kg]

	def compute_partials(self, inputs, partials):
		N_motor = self.options['N_motor']
		partials['Weights|MotorControllers', 'max_power'] = 49.9/398 * 1/1000 * 1/N_motor





