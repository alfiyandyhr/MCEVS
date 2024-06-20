import numpy as np
import openmdao.api as om

from MCEVS.Weights.Powerplant.Rotor import RotorWeight
from MCEVS.Weights.Powerplant.Motor import MotorWeight

class PowerplantWeight(om.Group):
	"""
	Calculates powerplant weight
		W_powerplant = W_rotors + W_motors
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
			pass
			# self.add_subsystem('rotor_weight',
			# 					RotorWeight(N_rotor=N_rotors_lift),
			# 					promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
			# 					promotes_outputs=['Weights|Rotors'])
			# self.add_subsystem('motor_weight',
			# 					MotorWeight(N_motor=N_rotors_lift), # one rotor typically has one motor
			# 					promotes_outputs=['Weights|Motors'])
			# self.connect('power_hover', 'motor_weight.max_power')   # assume max power output = power in hover
			# self.add_subsystem('ESC_weight',
			# 					ESCWeight(N_motor=N_rotors_lift), # one rotor typically has one motor
			# 					promotes_outputs=['eVTOL|W_ESC_all'])
			# self.connect('power_hover', 'ESC_weight.max_power')   # assume max power output = power in hover

		elif params['evtol_config'] == 'lift+cruise':
			# Lifting rotors
			self.add_subsystem('rotor_weight_lift',
								RotorWeight(N_rotor=N_rotors_lift),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_lift')])
			self.add_subsystem('motor_weight_lift',
								MotorWeight(N_motor=N_rotors_lift), # one rotor typically has one motor
								promotes_outputs=[('Weights|Motors', 'W_motors_lift')])

			# Cruising rotors
			self.add_subsystem('rotor_weight_cruise',
								RotorWeight(N_rotor=N_rotors_cruise),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_cruise')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_cruise')])
			self.add_subsystem('motor_weight_cruise',
								MotorWeight(N_motor=N_rotors_cruise), # one rotor typically has one motor
								promotes_outputs=[('Weights|Motors', 'W_motors_cruise')])

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
			self.add_subsystem('powerplant_weight',
								adder,
								promotes_inputs=['*'],
								promotes_outputs=[('W_rotors', 'Weights|Rotors'), ('W_motors', 'Weights|Motors')])

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weights|Powerplant',
							input_names=['Weights|Rotors', 'Weights|Motors'],
							units='kg',
							scaling_factors=[1., 1.])
		self.add_subsystem('powerplant_sum_weight',
							adder,
							promotes_inputs=['Weights|*'],
							promotes_outputs=['Weights|Powerplant'])




