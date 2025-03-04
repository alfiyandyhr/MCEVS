import numpy as np
import openmdao.api as om

class MotorWeight(om.ExplicitComponent):
	"""
	Computes motor weight
	Parameters:
		N_motor	: number of motors
		tf 		: technology factor
		PM 		: power margin
	Inputs:
		max_power : maximum power, i.e., power during climb [W]
	Outputs:
		Weight|motors : weight of all motors [kg]
	Notes:
		> An empirical equation based on power density regression on DC electric motors
		> It is suggested to renew the equation as more data become available
	Source:
		MCEVS/data/motors/motor_data.xlsx
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')
		self.options.declare('PM', types=float, desc='Power margin')
		self.options.declare('tf', types=float, desc='Technology factor')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weight|motors', units='kg', desc='Weight of all motors')
		self.declare_partials('Weight|motors', 'max_power')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		tf = self.options['tf']
		p_max = (1 + self.options['PM']/100) * inputs['max_power']/1000.0 # in [kW]

		# Calculating W_motors
		W_motor = 0.2138 * p_max/N_motor
		W_motors = N_motor * W_motor

		outputs['Weight|motors'] = tf * W_motors # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		partials['Weight|motors', 'max_power'] = tf * 0.2138/1000.0 * (1 + self.options['PM']/100)

class MotorWeightV1(om.ExplicitComponent):
	"""
	Computes motor weight
	Parameters:
		N_motor	: number of motors
	Inputs:
		max_power : maximum power, i.e., power during climb [W]
	Outputs:
		Weight|motors : weight of all motors [kg]
	Notes:
		> An empirical equation based on power density regression on DC electric motors
	Source:
		Ugwueze, O., et al., “Investigation of a Mission-Based Sizing Method for Electric VTOL Aircraft Preliminary Design,”
		presented at the AIAA SCITECH 2022 Forum, San Diego, CA & Virtual, 2022. https://doi.org/10.2514/6.2022-1931
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')

	def setup(self):
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weight|motors', units='kg', desc='Weight of all motors')
		self.declare_partials('Weight|motors', 'max_power')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		p_max = inputs['max_power']/1000.0 # in [kW]

		# Calculating W_motors
		W_motor = (0.188 * p_max + 5.836)/N_motor
		W_motors = N_motor * W_motor

		outputs['Weight|motors'] = W_motors # in [kg]

	def compute_partials(self, inputs, partials):
		partials['Weight|motors', 'max_power'] = 0.188/1000.0

class MotorWeightV2(om.ExplicitComponent):
	"""
	Computes motor weight
	Parameters:
		N_motor	: number of motors
	Inputs:
		max_torque : maximum torque, i.e., torque during hover [Nm]
	Outputs:
		Weight|motors : weight of all motors [kg]
	Notes:
	Source:
		1. Kadhiresan, A. R., and Duffy, M. J., “Conceptual Design and Mission Analysis for EVTOL Urban Air Mobility Flight Vehicle Configurations,”
		   presented at the AIAA Aviation 2019 Forum, Dallas, Texas, 2019. https://doi.org/10.2514/6.2019-2873
  		2. Duffy, M., Sevier, A. E., Hupp, R., Perdomo, E., and Wakayama, S., “Propulsion Scaling Methods in the Era of Electric Flight,”
  		   presented at the 2018 AIAA/IEEE Electric Aircraft Technologies Symposium, Cincinnati, Ohio, 2018. https://doi.org/10.2514/6.2018-4978
	"""
	def initialize(self):
		self.options.declare('N_motor', types=int, desc='Number of motors')

	def setup(self):
		self.add_input('max_torque', units='N*m', desc='Maximum torque')
		self.add_output('Weight|motors', units='kg', desc='Weight of all motors')
		self.declare_partials('Weight|motors', 'max_torque')

	def compute(self, inputs, outputs):
		N_motor = self.options['N_motor']
		tau_max = inputs['max_torque'] # in [N*m]

		# Calculating W_motors
		W_motor = 58/990 * (tau_max - 10.0) + 2.0
		W_motors = N_motor * W_motor

		outputs['Weight|motors'] = W_motors # in [kg]

	def compute_partials(self, inputs, partials):
		partials['Weight|motors', 'max_torque'] = 58/990





