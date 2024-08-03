import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Propulsion.Rotor import RotorWeight
from MCEVS.Analyses.Weight.Propulsion.Motor import MotorWeight
from MCEVS.Analyses.Weight.Propulsion.Controller import MotorControllerWeight

class PropulsionWeight(om.Group):
	"""
	Calculates propulsion weight
		W_propulsion = W_rotors + W_motors + W_controllers
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')

	def setup(self):

		vehicle = self.options['vehicle']

		N_lift_rotor 		= vehicle.lift_rotor.n_rotor				# number of lift rotors
		N_blade_lift_rotor 	= vehicle.lift_rotor.n_blade 				# number of blades per lift rotor
		tf_lift_rotor 		= vehicle.lift_rotor.technology_factor		# technology factor of lift rotors


		if vehicle.configuration == 'LiftPlusCruise':
			N_propeller 		= vehicle.propeller.n_propeller 		# number of propellers
			N_blade_propeller 	= vehicle.propeller.n_blade 			# number of blades per propeller
			tf_propeller 		= vehicle.propeller.technology_factor	# technology factor of propellers

		# For multirotor
		if vehicle.configuration == 'Multirotor':
			self.add_subsystem('lift_rotor_weight',
								RotorWeight(N_rotor=N_lift_rotor, N_bl=N_blade_lift_rotor, tf=tf_lift_rotor),
								promotes_inputs=[('Rotor|radius', 'LiftRotor|radius')],
								promotes_outputs=['Weight|rotors'])
			self.add_subsystem('lift_rotor_motor_weight',
								MotorWeight(N_motor=N_lift_rotor), 		# one rotor typically has one motor
								promotes_outputs=['Weight|motors'])
			self.add_subsystem('lift_rotor_controller_weight',
								MotorControllerWeight(N_motor=N_lift_rotor),
								promotes_outputs=['Weight|controllers']) 	# one motor has one controller

		elif vehicle.configuration == 'LiftPlusCruise':
			# Lift rotors
			self.add_subsystem('lift_rotor_weight',
								RotorWeight(N_rotor=N_lift_rotor, N_bl=N_blade_lift_rotor, tf=tf_lift_rotor),
								promotes_inputs=[('Rotor|radius', 'LiftRotor|radius')],
								promotes_outputs=[('Weight|rotors', 'W_rotors_lift_rotor')])
			self.add_subsystem('lift_rotor_motor_weight',
								MotorWeight(N_motor=N_lift_rotor), 		# one rotor typically has one motor
								promotes_outputs=[('Weight|motors', 'W_motors_lift_rotor')])
			self.add_subsystem('lift_rotor_controller_weight',
								MotorControllerWeight(N_motor=N_lift_rotor),
								promotes_outputs=[('Weight|controllers', 'W_controllers_lift_rotor')]) # one motor has one controller

			# Cruising rotors
			self.add_subsystem('propeller_weight',
								RotorWeight(N_rotor=N_propeller, N_bl=N_blade_propeller, tf=tf_propeller),
								promotes_inputs=[('Rotor|radius', 'Propeller|radius')],
								promotes_outputs=[('Weight|rotors', 'W_rotors_propeller')])
			self.add_subsystem('propeller_motor_weight',
								MotorWeight(N_motor=N_propeller), # one rotor typically has one motor
								promotes_outputs=[('Weight|motors', 'W_motors_propeller')])
			self.add_subsystem('propeller_controller_weight',
								MotorControllerWeight(N_motor=N_propeller),
								promotes_outputs=[('Weight|controllers', 'W_controllers_propeller')]) # one motor has one controller

			# Sum both systems weight
			adder = om.AddSubtractComp()
			adder.add_equation('W_rotors',
								input_names=['W_rotors_lift_rotor', 'W_rotors_propeller'],
								units='kg',
								scaling_factors=[1., 1.])
			adder.add_equation('W_motors',
								input_names=['W_motors_lift_rotor', 'W_motors_propeller'],
								units='kg',
								scaling_factors=[1., 1.])
			adder.add_equation('W_controllers',
								input_names=['W_controllers_lift_rotor', 'W_controllers_propeller'],
								units='kg',
								scaling_factors=[1., 1.])
			self.add_subsystem('propulsion_weight',
								adder,
								promotes_inputs=['*'],
								promotes_outputs=[('W_rotors', 'Weight|rotors'), ('W_motors', 'Weight|motors'), ('W_controllers', 'Weight|controllers')])

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weight|propulsion',
							input_names=['Weight|rotors', 'Weight|motors', 'Weight|controllers'],
							units='kg',
							scaling_factors=[1., 1., 1.])
		self.add_subsystem('propulsion_sum_weight',
							adder,
							promotes_inputs=['Weight|*'],
							promotes_outputs=['Weight|propulsion'])




