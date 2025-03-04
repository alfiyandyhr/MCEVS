import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Propulsion.Rotor import RotorAndHubWeight, ExtraHubWeight, ExtraHubWeightWithFixedVtip
from MCEVS.Analyses.Weight.Propulsion.Motor import MotorWeight
from MCEVS.Analyses.Weight.Propulsion.Controller import MotorControllerWeight

class PropulsionWeight(om.Group):
	"""
	Calculates propulsion weight
		W_propulsion = W_rotors_and_hubs + W_extra_hubs + W_motors + W_controllers
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('tf_propulsion', types=float, desc='Technology factor for propulsion systems')

	def setup(self):

		vehicle = self.options['vehicle']
		tf_propulsion = self.options['tf_propulsion']

		N_lift_rotor 		= vehicle.lift_rotor.n_rotor				# number of lift rotors
		N_blade_lift_rotor 	= vehicle.lift_rotor.n_blade 				# number of blades per lift rotor
		# tf_lift_rotor 		= vehicle.lift_rotor.technology_factor		# technology factor of lift rotors


		if vehicle.configuration == 'LiftPlusCruise':
			N_propeller 		= vehicle.propeller.n_propeller 		# number of propellers
			N_blade_propeller 	= vehicle.propeller.n_blade 			# number of blades per propeller
			# tf_propeller 		= vehicle.propeller.technology_factor	# technology factor of propellers

		# For multirotor
		if vehicle.configuration == 'Multirotor':
			self.add_subsystem('lift_rotor_weight',
								RotorAndHubWeight(N_rotor=N_lift_rotor, tf=tf_propulsion),
								promotes_outputs=['Weight|rotors_and_hubs'])
			self.add_subsystem('extra_hub_weight',
								ExtraHubWeight(N_rotor=N_lift_rotor, N_bl=N_blade_lift_rotor, tf=tf_propulsion),
								promotes_inputs=[('Weight|rotors','Weight|rotors_and_hubs'), ('Rotor|radius','LiftRotor|radius'), ('Rotor|chord', 'LiftRotor|chord'), ('Rotor|rpm','LiftRotor|HoverClimb|RPM')],
								promotes_outputs=['Weight|extra_hubs'])
			self.add_subsystem('lift_rotor_motor_weight',
								MotorWeight(N_motor=N_lift_rotor, PM=vehicle.lift_rotor.motor_power_margin, tf=tf_propulsion), # one rotor typically has one motor
								promotes_outputs=['Weight|motors'])
			self.add_subsystem('lift_rotor_controller_weight',
								MotorControllerWeight(N_motor=N_lift_rotor, PM=vehicle.lift_rotor.motor_power_margin, tf=tf_propulsion), # one motor has one controller
								promotes_outputs=['Weight|controllers']) 	
			
			input_names = ['Weight|rotors_and_hubs', 'Weight|extra_hubs', 'Weight|motors', 'Weight|controllers']
			sfs = [1., 1., 1., 1.]			

		elif vehicle.configuration == 'LiftPlusCruise':
			# Lift rotors
			self.add_subsystem('lift_rotor_weight',
								RotorAndHubWeight(N_rotor=N_lift_rotor, tf=tf_propulsion),
								promotes_outputs=[('Weight|rotors_and_hubs', 'W_rotors_and_hubs_lift_rotor')])
			self.add_subsystem('lift_rotor_motor_weight',
								MotorWeight(N_motor=N_lift_rotor, PM=vehicle.lift_rotor.motor_power_margin, tf=tf_propulsion), # one rotor typically has one motor
								promotes_outputs=[('Weight|motors', 'W_motors_lift_rotor')])
			self.add_subsystem('lift_rotor_controller_weight',
								MotorControllerWeight(N_motor=N_lift_rotor, PM=vehicle.lift_rotor.motor_power_margin, tf=tf_propulsion), # one motor has one controller
								promotes_outputs=[('Weight|controllers', 'W_controllers_lift_rotor')])

			# Cruising rotors
			self.add_subsystem('propeller_weight',
								RotorAndHubWeight(N_rotor=N_propeller, tf=tf_propulsion),
								promotes_outputs=[('Weight|rotors_and_hubs', 'W_rotors_and_hubs_propeller')])
			self.add_subsystem('propeller_motor_weight',
								MotorWeight(N_motor=N_propeller, PM=vehicle.propeller.motor_power_margin, tf=tf_propulsion), # one rotor typically has one motor
								promotes_outputs=[('Weight|motors', 'W_motors_propeller')])
			self.add_subsystem('propeller_controller_weight',
								MotorControllerWeight(N_motor=N_propeller, PM=vehicle.propeller.motor_power_margin, tf=tf_propulsion),
								promotes_outputs=[('Weight|controllers', 'W_controllers_propeller')]) # one motor has one controller

			# Sum both systems weight
			adder = om.AddSubtractComp()
			adder.add_equation('W_rotors_and_hubs',
								input_names=['W_rotors_and_hubs_lift_rotor', 'W_rotors_and_hubs_propeller'],
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
								promotes_outputs=[('W_rotors_and_hubs', 'Weight|rotors_and_hubs'), ('W_motors', 'Weight|motors'), ('W_controllers', 'Weight|controllers')])

			input_names = ['Weight|rotors_and_hubs', 'Weight|motors', 'Weight|controllers']
			sfs = [1., 1., 1.]


		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weight|propulsion',
							input_names=input_names,
							units='kg',
							scaling_factors=sfs)
		self.add_subsystem('propulsion_sum_weight',
							adder,
							promotes_inputs=['Weight|*'],
							promotes_outputs=['Weight|propulsion'])




