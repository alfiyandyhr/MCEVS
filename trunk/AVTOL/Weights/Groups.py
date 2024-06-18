import numpy as np
import openmdao.api as om

from AVTOL.Energy.Energy import EnergyConsumption

from AVTOL.Weights.Powerplant.Rotor import RotorWeight
from AVTOL.Weights.Powerplant.Motor import MotorWeight


# --- Total weight (MTOW) estimation --- #

class MTOWEstimation(om.Group):
	"""
	Computes UAV total weight estimation given the design variables and mission requirement.
	Must be used with an optimizer (or a nonlinear solver) to converge the weight residual.

	Inputs:
	(mission requirements)
		flight_distance
		hover_time
		payload_weight
	(UAV design variables)
		eVTOL|W_takeoff 		: total take-off weight [kg]
		eVTOL|Cruise_speed 		: cruising speed of the eVTOL [m/s]
		Rotor|radius_lift		: Lifting rotor radius
		Rotor|radius_cruise 	: Cruising rotor radius 	(for lift+cruise only)
		eVTOL|S_wing			: Wing area 				(for lift+cruise only)
		Rotor|mu 				: Rotor advance ratio		(for multirotor only)
		Rotor|J 				: Propeller advance ratio	(for lift+cruise only)

	Outputs:
	(weight of each component)
		eVTOL|W_battery
		Weights|Rotors
		Weights|Motors
		eVTOL|W_ESC_all
	(major performances)
		power_hover
		power_cruise
	(for some constraints)
		disk_loading_hover
		disk_loading_cruise
		Rotor|Ct (in cruise)
		CL_cruise
	"""

	def initialize(self):
		self.options.declare('evtol_options', types=dict, desc='Dict containing all options parameters')
		self.options.declare('use_solver', types=bool, desc='Whether to use non linear solver')

	def setup(self):
		params = self.options['evtol_options']
		use_solver = self.options['use_solver']

		# Unpacking options
		eVTOL_config = params['evtol_config']
		battery_rho = params['battery_rho']
		battery_eff = params['battery_eff']
		battery_max_discharge = params['battery_max_discharge']
		N_rotors_lift = params['N_rotors_lift']		# number of lifting rotors

		if eVTOL_config == 'multirotor':
			pass
		elif eVTOL_config == 'lift+cruise':
			N_rotors_cruise = params['N_rotors_cruise']
		else:
			raise RuntimeError('eVTOL configuration is not available')

		# --- Calculate total energy consumptions to fly the mission --- #
		self.add_subsystem('energy',
							EnergyConsumption(evtol_options=params),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# --- Calculate weight estimation of each component --- #

		# 1. Battery weight
		# battery weight is computed taking into account loss in efficiency,
		# avionics power, and its maximum discharge rate
		battery_weight_comp = om.ExecComp('W_battery = energy_req / (battery_rho * battery_eff * battery_max_discharge)',
										   W_battery={'units': 'kg'},
										   energy_req={'units': 'W * h'},
										   battery_rho={'units': 'W * h / kg', 'val': battery_rho},
										   battery_eff={'val': battery_eff},
										   battery_max_discharge={'val': battery_max_discharge})
		self.add_subsystem('battery_weight',
							battery_weight_comp,
							promotes_inputs=[('energy_req', 'energy_cnsmp')],
							promotes_outputs=[('W_battery', 'Weights|Battery')])

		# 2. Powerplant weight
		# includes the weight of rotors, motors, and ESCs

		# For multirotor
		if eVTOL_config == 'multirotor':
			self.add_subsystem('rotor_weight',
								RotorWeight(N_rotor=N_rotors_lift),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
								promotes_outputs=['Weights|Rotors'])
			self.add_subsystem('motor_weight',
								MotorWeight(N_motor=N_rotors_lift), # one rotor typically has one motor + ESC
								promotes_outputs=['Weights|Motors'])
			self.connect('power_hover', 'motor_weight.max_power')   # assume max power output = power in hover
			self.add_subsystem('ESC_weight',
								ESCWeight(N_motor=N_rotors_lift), # one rotor typically has one motor + ESC
								promotes_outputs=['eVTOL|W_ESC_all'])
			self.connect('power_hover', 'ESC_weight.max_power')   # assume max power output = power in hover

		elif eVTOL_config == 'lift+cruise':
			# Lifting rotors
			self.add_subsystem('rotor_weight_lift',
								RotorWeight(N_rotor=N_rotors_lift),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_lift')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_lift')])
			self.connect('power_hover', 'rotor_weight_lift.max_power')
			self.add_subsystem('motor_weight_lift',
								MotorWeight(N_motor=N_rotors_lift), # one rotor typically has one motor + ESC
								promotes_outputs=[('Weights|Motors', 'W_motors_lift')])
			self.connect('power_hover', 'motor_weight_lift.max_power')   # assume max power output = power in hover
			# self.add_subsystem('ESC_weight_lift',
			# 					ESCWeight(N_motor=N_rotors_lift), # one rotor typically has one motor + ESC
			# 					promotes_outputs=[('eVTOL|W_ESC_all', 'W_ESCs_lift')])
			# self.connect('power_hover', 'ESC_weight_lift.max_power')   # assume max power output = power in hover

			# Cruising rotors
			self.add_subsystem('rotor_weight_cruise',
								RotorWeight(N_rotor=N_rotors_cruise),
								promotes_inputs=[('Rotor|radius', 'Rotor|radius_cruise')],
								promotes_outputs=[('Weights|Rotors', 'W_rotors_cruise')])
			self.connect('power_forward', 'rotor_weight_cruise.max_power')
			self.add_subsystem('motor_weight_cruise',
								MotorWeight(N_motor=N_rotors_cruise), # one rotor typically has one motor + ESC
								promotes_outputs=[('Weights|Motors', 'W_motors_cruise')])
			self.connect('power_forward', 'motor_weight_cruise.max_power')   # assume max power output = power in cruise
			# self.add_subsystem('ESC_weight_cruise',
			# 					ESCWeight(N_motor=N_rotors_cruise), # one rotor typically has one motor + ESC
			# 					promotes_outputs=[('eVTOL|W_ESC_all', 'W_ESCs_cruise')])
			# self.connect('power_forward', 'ESC_weight_cruise.max_power')   # assume max power output = power in cruise

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
		# 	adder.add_equation('W_ESCs',
		# 						input_names=['W_ESCs_lift', 'W_ESCs_cruise'],
		# 						units='kg',
		# 						scaling_factors=[1., 1.])
			self.add_subsystem('powerplant_weight',
								adder,
								promotes_inputs=['*'],
								# promotes_outputs=[('W_rotors', 'Weights|Rotors'), ('W_motors', 'Weights|Motors'), ('W_ESCs', 'eVTOL|W_ESC_all')])
								promotes_outputs=[('W_rotors', 'Weights|Rotors'), ('W_motors', 'Weights|Motors')])

		# # 3. Wing weight
		# # wing is possessed by a lift+cruise configuration
		# if eVTOL_config == 'lift+cruise':
		# 	self.add_subsystem('wing_weight',
		# 						WingWeight(),
		# 						promotes_inputs=['eVTOL|S_wing'],
		# 						promotes_outputs=['eVTOL|W_wing'])

		# # 4. Weight residuals
		# # W_residual = W_total - W_battery - W_payload - W_wing - W_propulsion - W_frame
		# # where:
		# # W_propulsion = W_rotor_all + W_motor_all + W_ESC_all
		# # and
		# # W_frame = 0.2 * W_total + 0.5 (kg)
		# # W_residual should then be driven to 0 by a nonlinear solver or treated as an optimization constraint
		# input_list = [('W_total', 'eVTOL|W_takeoff'),
		# 			  ('W_battery', 'eVTOL|W_battery'),
		# 			  ('W_payload', 'payload_weight'),
		# 			  ('W_wing', 'eVTOL|W_wing'),
		# 			  ('W_rotor_all', 'Weights|Rotors'),
		# 			  ('W_motor_all', 'Weights|Motors'),
		# 			  ('W_ESC_all', 'eVTOL|W_ESC_all')] 

		# W_residual_eqn = 'W_residual = W_total - W_battery - W_payload - W_wing - W_rotor_all - W_motor_all - W_ESC_all - 0.2*W_total - 0.5'
		# self.add_subsystem('w_residual_comp',
		# 					om.ExecComp(W_residual_eqn, units='kg'),
		# 					promotes_inputs=input_list,
		# 					promotes_outputs=['W_residual'])
		# # for wingless multirotor, set 0 weight for wing
		# if eVTOL_config == 'multirotor':
		# 	self.set_input_defaults('eVTOL|W_wing', 0.0)

		# # If nonlinear solver to be used
		# if use_solver:
		# 	# This drives W_residual = 0 by varying W_total. LB and UB of W_total should be given.
		# 	residual_balance = om.BalanceComp('W_total',
		# 									   units='kg',
		# 									   eq_units='kg',
		# 									   lower=1.0,
		# 									   upper=10.0,
		# 									   val=5.0,
		# 									   rhs_val=0.0,
		# 									   use_mult=False)
		# 	self.add_subsystem('weight_balance',
		# 						residual_balance,
		# 						promotes_inputs=[('lhs:W_total', 'W_residual')],
		# 						promotes_outputs=[('W_total', 'eVTOL|W_takeoff')])

		# 	# self.connect('weight_balance.W_total', 'eVTOL|W_takeoff')
		# 	# self.connect('w_residual_comp.W_residual', 'weight_balance.lhs:W_total')

		# 	self.set_input_defaults('weight_balance.rhs:W_total', 0.0)

		# 	# Add solvers for implicit relations
		# 	self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=30, iprint=0, rtol=1e-10)
		# 	self.nonlinear_solver.options['err_on_non_converge'] = True
		# 	self.nonlinear_solver.options['reraise_child_analysiserror'] = True
		# 	self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
		# 	self.nonlinear_solver.linesearch.options['maxiter'] = 10
		# 	self.nonlinear_solver.linesearch.options['iprint'] = 0
		# 	self.linear_solver = om.DirectSolver(assemble_jac=True)