import numpy as np
import openmdao.api as om

class MotorControllerWeight(om.ExplicitComponent):
	"""
	Computes motor controller weight
	Parameters:
		N_motor							: number of motors
		tf 								: technology factor
		PM 								: power margin
	Inputs:
		max_power 						: maximum power, i.e., power during climb [W]
	Outputs:
		Weight|propulsion|controllers 	: weight of all motor controllers [kg]
	Notes:
	Sources:
  		Duffy, M., Sevier, A. E., Hupp, R., Perdomo, E., and Wakayama, S., “Propulsion Scaling Methods in the Era of Electric Flight,”
  		presented at the 2018 AIAA/IEEE Electric Aircraft Technologies Symposium, Cincinnati, Ohio, 2018. https://doi.org/10.2514/6.2018-4978
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')
		self.options.declare('PM', types=float, desc='Power margin')
		self.options.declare('tf', types=float, desc='Technology factor')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weight|propulsion|controllers', units='kg', desc='Weight of all motor controllers')
		self.declare_partials('Weight|propulsion|controllers', 'max_power')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		tf = self.options['tf']
		p_max = (1 + self.options['PM']/100) * inputs['max_power']/1000.0 # in [kW]

		# Calculating W_controllers
		W_controller = 0.1149 * (p_max/N_motor)
		W_controllers = N_motor * W_controller

		outputs['Weight|propulsion|controllers'] = tf * W_controllers # in [kg]

	def compute_partials(self, inputs, partials):
		N_motor = self.options['N_motor']
		tf = self.options['tf']
		partials['Weight|propulsion|controllers', 'max_power'] = tf * 0.1149/1000 * (1 + self.options['PM']/100)





