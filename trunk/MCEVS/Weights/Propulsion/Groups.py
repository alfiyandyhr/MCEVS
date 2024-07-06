import numpy as np
import openmdao.api as om

from MCEVS.Weights.Propulsion.Rotor import RotorWeight
from MCEVS.Weights.Propulsion.Motor import MotorWeightSimple
from MCEVS.Weights.Propulsion.Controller import MotorControllerWeight

class PropulsionWeight(om.Group):
	"""
	Calculates propulsion weight
		W_propulsion = W_rotors + W_motors + W_controllers
	"""
	def initialize(self):
		self.options.declare('params', types=dict, desc='A dictionary that includes important parameters')

	def setup(self):
		params = self.options['params']
		N_rotors_lift = params['N_rotors_lift']		# number of lifting rotors

		if params['evtol_config'] == 'multirotor':
			pass
		elif params['evtol_config'] == 'lift+cruise':
			N_rotors_cruise = params['N_rotors_cruise']
		else:
			raise RuntimeError('eVTOL configuration is not available')

		# For multirotor
		if params['evtol_config'] == 'multirotor':
			self.add_subsystem('rotor_weight',
								RotorWeight(N_rotor=N_rotors_lift),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
								promotes_outputs=['Weights|Rotors'])
			self.add_subsystem('motor_weight',
								MotorWeightSimple(N_motor=N_rotors_lift), # one rotor typically has one motor
								promotes_outputs=['Weights|Motors'])
			self.add_subsystem('controller_weight',
								MotorControllerWeight(N_motor=N_rotors_lift),
								promotes_outputs=['Weights|MotorControllers']) # one motor has one controller

		elif params['evtol_config'] == 'lift+cruise':
			# Lifting rotors
			self.add_subsystem('rotor_weight_lift',
								RotorWeight(N_rotor=N_rotors_lift),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_lift')])
			self.add_subsystem('motor_weight_lift',
								MotorWeightSimple(N_motor=N_rotors_lift), # one rotor typically has one motor
								promotes_outputs=[('Weights|Motors', 'W_motors_lift')])
			self.add_subsystem('controller_weight_lift',
								MotorControllerWeight(N_motor=N_rotors_lift),
								promotes_outputs=[('Weights|MotorControllers', 'W_controllers_lift')]) # one motor has one controller

			# Cruising rotors
			self.add_subsystem('rotor_weight_cruise',
								RotorWeight(N_rotor=N_rotors_cruise),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_cruise')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_cruise')])
			self.add_subsystem('motor_weight_cruise',
								MotorWeightSimple(N_motor=N_rotors_cruise), # one rotor typically has one motor
								promotes_outputs=[('Weights|Motors', 'W_motors_cruise')])
			self.add_subsystem('controller_weight_cruise',
								MotorControllerWeight(N_motor=N_rotors_cruise),
								promotes_outputs=[('Weights|MotorControllers', 'W_controllers_cruise')]) # one motor has one controller

			# Sum both systems weight
			adder = om.AddSubtractComp()
			adder.add_equation('W_rotors',
								input_names=['W_rotors_lift', 'W_rotors_cruise'],
								units='kg',
								scaling_factors=[1., 1.])
			adder.add_equation('W_motors',
								input_names=['W_motors_lift', 'W_motors_cruise'],
								units='kg',
								scaling_factors=[1., 1.])
			adder.add_equation('W_controllers',
								input_names=['W_controllers_lift', 'W_controllers_cruise'],
								units='kg',
								scaling_factors=[1., 1.])
			self.add_subsystem('propulsion_weight',
								adder,
								promotes_inputs=['*'],
								promotes_outputs=[('W_rotors', 'Weights|Rotors'), ('W_motors', 'Weights|Motors'), ('W_controllers', 'Weights|MotorControllers')])

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weights|Propulsion',
							input_names=['Weights|Rotors', 'Weights|Motors', 'Weights|MotorControllers'],
							units='kg',
							scaling_factors=[1., 1., 1.])
		self.add_subsystem('propulsion_sum_weight',
							adder,
							promotes_inputs=['Weights|*'],
							promotes_outputs=['Weights|Propulsion'])




