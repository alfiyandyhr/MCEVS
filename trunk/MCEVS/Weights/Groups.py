import numpy as np
import openmdao.api as om

from MCEVS.Energy.Energy import EnergyConsumption

from MCEVS.Weights.Propulsion.Groups import PropulsionWeight
from MCEVS.Weights.Structure.Groups import StructureWeight
from MCEVS.Weights.Equipment.Groups import EquipmentWeight


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
	(eVTOL design variables)
		eVTOL|W_takeoff 		: total take-off weight [kg]
		eVTOL|Cruise_speed 		: cruising speed of the eVTOL [m/s]
		Rotor|radius_lift		: Lifting rotor radius
		Rotor|radius_cruise 	: Cruising rotor radius 	(for lift+cruise only)
		eVTOL|S_wing			: Wing area 				(for lift+cruise only)
		eVTOL|AR_wing			: wing aspect ratio 		(for lift+cruise only)
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

		# --- Calculate total energy consumptions to fly the mission --- #
		self.add_subsystem('energy',
							EnergyConsumption(evtol_options=params),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# # --- Calculate weight estimation of each component --- #

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

		# 2. Propulsion weight
		# includes the weight of rotors, motors, and ESCs
		self.add_subsystem('propulsion_weight',
							PropulsionWeight(params=params),
							promotes_inputs=['*'],
							promotes_outputs=['*'])
		if eVTOL_config == 'multirotor':
			self.connect('power_hover', 'rotor_weight.max_power')
			self.connect('power_hover', 'motor_weight.max_power')  					# assume max power output = power in hover
			self.connect('power_hover', 'controller_weight.max_power') 				# assume max power output = power in hover
		elif eVTOL_config == 'lift+cruise':
			self.connect('power_hover', 'rotor_weight_lift.max_power')
			self.connect('power_hover', 'motor_weight_lift.max_power')				# assume max power output = power in hover
			self.connect('power_hover', 'controller_weight_lift.max_power')			# assume max power output = power in hover
			self.connect('power_forward', 'rotor_weight_cruise.max_power')
			self.connect('power_forward', 'motor_weight_cruise.max_power')   		# assume max power output = power in cruise
			self.connect('power_forward', 'controller_weight_cruise.max_power')		# assume max power output = power in cruise

		# 3. Structure weight
		# includes fuselage, landing gear, wing, and tails
		if eVTOL_config == 'multirotor': input_list_struct = ['eVTOL|W_takeoff']
		elif eVTOL_config == 'lift+cruise': input_list_struct = ['eVTOL|W_takeoff', 'eVTOL|S_wing', 'eVTOL|AR_wing']
		self.add_subsystem('structure_weight',
							StructureWeight(params=params),
							promotes_inputs=input_list_struct,
							promotes_outputs=['Weights|*'])

		# 4. Equipment weight
		# includes avionics, flight control, anti icing, and furnishing
		self.add_subsystem('equipment_weight',
							EquipmentWeight(),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|*'])


		# 4. Weight residuals
		
		# W_residual = W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment
		# where:
		# W_propulsion = W_rotors + W_motors + W_controllers
		# W_structure = W_fuselage + W_landing_gear + W_wing
		# W_equipment = W_avionics + W_flight_control + W_anti_icing + W_furnishings

		# W_residual should then be driven to 0 by a nonlinear solver or treated as an optimization constraint
		input_list = [('W_total', 'eVTOL|W_takeoff'),
					  ('W_payload', 'payload_weight'),
					  ('W_battery', 'Weights|Battery'),
					  ('W_propulsion', 'Weights|Propulsion'),
					  ('W_structure', 'Weights|Structure'),
					  ('W_equipment', 'Weights|Equipment')]

		W_residual_eqn = 'W_residual = W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment'
		self.add_subsystem('w_residual_comp',
							om.ExecComp(W_residual_eqn, units='kg'),
							promotes_inputs=input_list,
							promotes_outputs=['W_residual'])

		# If nonlinear solver to be used
		if use_solver:
			# This drives W_residual = 0 by varying W_total. LB and UB of W_total should be given.
			residual_balance = om.BalanceComp('W_total',
											   units='kg',
											   eq_units='kg',
											   lower=500.0,
											   upper=2500.0,
											   val=1500.0,
											   rhs_val=0.0,
											   use_mult=False)
			self.add_subsystem('weight_balance',
								residual_balance,
								promotes_inputs=[('lhs:W_total', 'W_residual')],
								promotes_outputs=[('W_total', 'eVTOL|W_takeoff')])

			# self.connect('weight_balance.W_total', 'w_residual_comp.W_total')
			# self.connect('w_residual_comp.W_residual', 'weight_balance.lhs:W_total')

			self.set_input_defaults('weight_balance.rhs:W_total', 0.0)

			# Add solvers for implicit relations
			self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=50, iprint=0, rtol=1e-10)
			self.nonlinear_solver.options['err_on_non_converge'] = True
			self.nonlinear_solver.options['reraise_child_analysiserror'] = True
			self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
			self.nonlinear_solver.linesearch.options['maxiter'] = 10
			self.nonlinear_solver.linesearch.options['iprint'] = 0
			self.linear_solver = om.DirectSolver(assemble_jac=True)


