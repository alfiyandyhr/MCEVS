import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Energy.Analysis import EnergyConsumption
from MCEVS.Analyses.Weight.Battery.Groups import BatteryWeight
from MCEVS.Analyses.Weight.Propulsion.Groups import PropulsionWeight
from MCEVS.Analyses.Weight.Structure.Groups import StructureWeight
from MCEVS.Analyses.Weight.Equipment.Groups import EquipmentWeight
from MCEVS.Analyses.Geometry.Rotor import MeanChord
from MCEVS.Analyses.Geometry.Clearance import LiftRotorClearanceConstraint

from MCEVS.Utils.Performance import record_performance_by_segments

class VehicleWeight(object):
	"""
	docstring for VehicleWeight
	Weight of the vehicle.

	Top level info:
		> Max take-off weight
		> Payload weight
		> Battery weight
		> Propulsion weight
		> Structure weight
		> Equipment weight

	Broken down into:
		> Payload
		> Battery
		> Propulsion:
			- Motor
			- Controller
			- Rotors
		> Structure:
			- Fuselage
			- Landing_Gear
			- Empennage
			- Wing (if any)
		> Equipment:
			- Anti-icing
			- Avionics
			- Flight_Control
			- Furnishings 
	"""
	def __init__(self, mtow=None):
		super(VehicleWeight, self).__init__()
		self.max_takeoff = mtow	# defined, or sized
		self.payload = None
		self.battery = None
		self.propulsion = None
		self.structure = None
		self.equipment = None

	def is_being_sized(self):
		pass

class WeightAnalysis(object):
	"""
	docstring for WeightAnalysis
	"""
	def __init__(self, vehicle:object, mission:object, fidelity:dict, sizing_mode=True, solved_by='optimization'):
		super(WeightAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.fidelity = fidelity
		self.sizing_mode = sizing_mode
		self.solved_by = solved_by
		if self.solved_by not in ['optimization', 'nonlinear_solver']:
			raise NotImplementedError('"solved_by" should be either "optimization" or "nonlinear_solver"')

	def evaluate(self, record=False, mtow_guess=None):

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			mean_c_to_R_lift_rotor 	= self.vehicle.lift_rotor.mean_c_to_R
			global_twist_lift_rotor = self.vehicle.lift_rotor.global_twist  # deg

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			mean_c_to_R_lift_rotor 	= self.vehicle.lift_rotor.mean_c_to_R
			global_twist_lift_rotor = self.vehicle.lift_rotor.global_twist  # deg
			r_propeller 			= self.vehicle.propeller.radius 		# m
			mean_c_to_R_propeller 	= self.vehicle.propeller.mean_c_to_R
			wing_area 				= self.vehicle.wing.area 				# m**2
			wing_aspect_ratio 		= self.vehicle.wing.aspect_ratio

		# --- OpenMDAO probolem --- #
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		for segment in self.mission.segments:
			if segment.kind == 'HoverClimbConstantSpeed':
				indeps.add_output('Mission|hover_climb_speed', segment.speed, units='m/s')
				indeps.add_output('LiftRotor|HoverClimb|RPM', self.vehicle.lift_rotor.RPM['hover_climb'], units='rpm')
			if segment.kind == 'HoverDescentConstantSpeed':
				indeps.add_output('Mission|hover_descent_speed', segment.speed, units='m/s')
			if segment.kind == 'CruiseConstantSpeed':
				indeps.add_output('Mission|cruise_speed', segment.speed, units='m/s')
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Cruise|RPM', self.vehicle.lift_rotor.RPM['cruise'], units='rpm')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Cruise|RPM', self.vehicle.propeller.RPM['cruise'], units='rpm')
			if segment.kind == 'ClimbConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Climb|RPM', self.vehicle.lift_rotor.RPM['climb'], units='rpm')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Climb|RPM', self.vehicle.propeller.RPM['climb'], units='rpm')		
			if segment.kind == 'DescentConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Descent|RPM', self.vehicle.lift_rotor.RPM['descent'], units='rpm')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Descent|RPM', self.vehicle.propeller.RPM['descent'], units='rpm')

		if self.vehicle.configuration == 'Multirotor':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')

		elif self.vehicle.configuration == 'LiftPlusCruise':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')
			indeps.add_output('Propeller|radius', r_propeller, units='m')
			indeps.add_output('Propeller|mean_c_to_R', mean_c_to_R_propeller, units=None)
			indeps.add_output('Wing|area', wing_area, units='m**2')
			indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)

		# Variables needed for BEMT
		if self.fidelity['hover_climb'] == 2:
			n_sections = self.vehicle.lift_rotor.n_section
			r_to_R_list = self.vehicle.lift_rotor.r_to_R_list
			c_to_R_list = self.vehicle.lift_rotor.c_to_R_list
			w_to_R_list = self.vehicle.lift_rotor.w_to_R_list
			if self.vehicle.lift_rotor.pitch_linear_grad is not None:
				indeps.add_output(f'LiftRotor|pitch_linear_grad', self.vehicle.lift_rotor.pitch_linear_grad, units='deg')
			else:
				pitch_list = np.array(self.vehicle.lift_rotor.pitch_list)
				for i in range(n_sections):
					indeps.add_output(f'LiftRotor|Section{i+1}|pitch', pitch_list[i], units='deg')
			for i in range(n_sections):
				indeps.add_output(f'LiftRotor|Section{i+1}|r_to_R', r_to_R_list[i], units=None)
				indeps.add_output(f'LiftRotor|Section{i+1}|c_to_R', c_to_R_list[i], units=None)
				indeps.add_output(f'LiftRotor|Section{i+1}|w_to_R', w_to_R_list[i], units=None)
		
		# Geometric analysis
		if self.vehicle.configuration == 'Multirotor':
			# Convert mean_c_to_R into mean_chord
			prob.model.add_subsystem('chord_calc_lift_rotor',
									  MeanChord(),
									  promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
									  promotes_outputs=[('mean_chord', 'LiftRotor|chord')])

		elif self.vehicle.configuration == 'LiftPlusCruise':
			# Calculate spanwise clearance constraint for lift rotor of LPC config
			prob.model.add_subsystem('lift_rotor_clearance',
									  LiftRotorClearanceConstraint(N_rotor=self.vehicle.lift_rotor.n_rotor,
									  							   max_d_fuse=self.vehicle.fuselage.max_diameter,
									  							   percent_max_span=95.0),
									  promotes_inputs=['LiftRotor|radius', 'Wing|area', 'Wing|aspect_ratio'],
									  promotes_outputs=[('clearance_constraint', 'LiftRotor|clearance_constraint')])
			# Convert mean_c_to_R into mean_chord
			prob.model.add_subsystem('chord_calc_lift_rotor',
									  MeanChord(),
									  promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
									  promotes_outputs=[('mean_chord', 'LiftRotor|chord')])
			prob.model.add_subsystem('chord_calc_propeller',
									  MeanChord(),
									  promotes_inputs=[('mean_c_to_R', 'Propeller|mean_c_to_R'), ('R', 'Propeller|radius')],
									  promotes_outputs=[('mean_chord', 'Propeller|chord')])

		# Core weight module
		prob.model.add_subsystem('mtow_model',
								  MTOWEstimation(mission=self.mission,
								  				 vehicle=self.vehicle,
								  				 fidelity=self.fidelity,
								  				 sizing_mode=False if self.solved_by=='optimization' else self.sizing_mode),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		# Sizing or not sizing
		if not self.sizing_mode:
			mtow_guess = 1500.0 if mtow_guess is None else mtow_guess
			indeps.add_output('Weight|takeoff', mtow_guess, units='kg') # mtow initial guess
			prob.setup(check=False)
			# prob.check_partials(compact_print=True, method='fd')
			prob.run_model()

		else:
			if self.solved_by == 'optimization':
				prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
				mtow_guess = 1500.0 if mtow_guess is None else mtow_guess
				indeps.add_output('Weight|takeoff', mtow_guess, units='kg') # mtow initial guess

				if self.fidelity['hover_climb'] in [0,1]:
					prob.model.add_design_var('Weight|takeoff', lower=600, upper=10000)
					prob.model.add_objective('Weight|residual')
					prob.setup(check=False)
					prob.run_driver()

				elif self.fidelity['hover_climb'] == 2:
					prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10, upper=5000)
					prob.model.add_design_var('Weight|takeoff', lower=600, upper=10000)
					prob.model.add_design_var('LiftRotor|global_twist', lower=0.0, upper=100.0)
					prob.model.add_objective('Weight|residual')
					prob.model.add_constraint('LiftRotor|HoverClimb|thrust_residual_square', lower=0, upper=0.1)
					prob.model.add_constraint('LiftRotor|HoverClimb|FM', lower=0.5, upper=0.8)
					prob.setup(check=False)
					prob.run_driver()

			elif self.solved_by == 'nonlinear_solver':

				if self.fidelity['hover_climb'] == 0:
					prob.setup(check=False)
					prob.run_model()

				elif self.fidelity['hover_climb'] == 2:
					prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
					prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10, upper=5000)
					prob.model.add_objective('LiftRotor|HoverClimb|thrust_residual_square')
					prob.setup(check=False)
					prob.run_driver()

		if record:
			record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

		return prob

class MTOWEstimation(om.Group):
	"""
	Computes eVTOL total weight estimation given the design variables and mission requirement.
	Must be used with an optimizer (or a nonlinear solver) to converge the weight residual.

	Inputs:
	(mission requirements)
		Mission object 			: an object containing mission profile information
	(vehicle definition)
		Vehicle object 			: an object that defines the vehicle
	(eVTOL design variables)
		Weight|takeoff 			: total take-off weight [kg]
		Mission|cruise_speed 	: cruising speed of the eVTOL [m/s]
		LiftRotor|radius		: Lifting rotor radius
		Propeller|radius 		: Cruising rotor radius 	(for lift+cruise only)
		Wing|area				: Wing area 				(for lift+cruise only)
		Wing|aspect_ratio		: wing aspect ratio 		(for lift+cruise only)
		LiftRotor|advance_ratio : Rotor advance ratio		(for multirotor only)
		Propeller|advance_ratio : Propeller advance ratio	(for lift+cruise only)

	Outputs:
	(weight of each component)
		Weight|battery
		Weight|propulsion
		Weight|structure
		Weight|equipment
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
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('sizing_mode', types=bool, desc='Whether to use in a sizing mode')

	def setup(self):
		
		# Unpacking option objects
		mission 	 = self.options['mission']
		vehicle 	 = self.options['vehicle']
		fidelity 	 = self.options['fidelity']
		sizing_mode  = self.options['sizing_mode']

		# Unpacking battery parameters
		battery_rho 			= vehicle.battery.density
		battery_eff 			= vehicle.battery.efficiency
		battery_max_discharge 	= vehicle.battery.max_discharge

		# --- Calculate total energy consumptions to fly the mission --- #
		self.add_subsystem('energy',
							EnergyConsumption(mission=mission,
											  vehicle=vehicle,
											  fidelity=fidelity),
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
							promotes_outputs=[('W_battery', 'Weight|battery')])
		self.connect('Energy|entire_mission', 'battery_weight.energy_req')

		# Alternative approach
		# self.add_subsystem('battery_weight',
		# 					BatteryWeight(battery_rho=battery_rho, battery_eff=battery_eff, battery_max_discharge=battery_max_discharge),
		# 					promotes_inputs=[('required_energy', 'Energy|entire_mission')],
		# 					promotes_outputs=['Weight|battery'])

		# 2. Propulsion weight
		# includes the weight of rotors, motors, and controllers

		self.add_subsystem('propulsion_weight',
							PropulsionWeight(vehicle=vehicle, tf_propulsion=vehicle.tf_propulsion),
							promotes_inputs=['*'],
							promotes_outputs=['*'])
		if vehicle.configuration == 'Multirotor':
			self.connect('LiftRotor|thrust_each|maximum', 'lift_rotor_weight.max_thrust')
			self.connect('Power|LiftRotor|maximum', 'lift_rotor_motor_weight.max_power')
			self.connect('Power|LiftRotor|maximum', 'lift_rotor_controller_weight.max_power')
		elif vehicle.configuration == 'LiftPlusCruise':
			self.connect('LiftRotor|thrust_each|maximum', 'lift_rotor_weight.max_thrust')
			self.connect('Power|LiftRotor|maximum', 'lift_rotor_motor_weight.max_power')
			self.connect('Power|LiftRotor|maximum', 'lift_rotor_controller_weight.max_power')
			self.connect('Propeller|thrust_each|maximum', 'propeller_weight.max_thrust')
			self.connect('Power|Propeller|maximum', 'propeller_motor_weight.max_power')
			self.connect('Power|Propeller|maximum', 'propeller_controller_weight.max_power')

		# 3. Structure weight
		# includes fuselage, landing gear, wing, and tails
		if vehicle.configuration == 'Multirotor': input_list_struct = ['Weight|takeoff']
		elif vehicle.configuration == 'LiftPlusCruise': input_list_struct = ['Weight|takeoff', 'Wing|area', 'Wing|aspect_ratio']
		self.add_subsystem('structure_weight',
							StructureWeight(vehicle=vehicle, tf_structure=vehicle.tf_structure),
							promotes_inputs=input_list_struct,
							promotes_outputs=['Weight|*'])
		# includes mission segment power for takeoff for sizing booms
		for i,segment in enumerate(mission.segments):
			if segment.kind in ['HoverClimbConstantSpeed']:
				self.connect(f'Power|LiftRotor|segment_{i+1}','structure_weight.total_req_takeoff_power')

		# 4. Equipment weight
		# includes avionics, flight control, anti icing, and furnishing
		self.add_subsystem('equipment_weight',
							EquipmentWeight(tf_equipment=vehicle.tf_equipment),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|*'])

		# 5. Weight residuals
		
		# W_residual = W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment
		# where:
		# W_propulsion = W_rotors + W_motors + W_controllers
		# W_structure = W_fuselage + W_landing_gear + W_wing
		# W_equipment = W_avionics + W_flight_control + W_anti_icing + W_furnishings

		# Payload weight
		# 1 PAX = 100.0 kg (default)
		n_pax = vehicle.fuselage.number_of_passenger
		payload_per_pax = vehicle.fuselage.payload_per_pax
		payload_weight = om.IndepVarComp('Weight|payload', val=n_pax*payload_per_pax, units='kg')
		self.add_subsystem('payload', payload_weight, promotes=['*'])

		# W_residual should then be driven to 0 by a nonlinear solver or treated as an optimization constraint
		input_list = [('W_total', 		'Weight|takeoff'),
					  ('W_payload', 	'Weight|payload'),
					  ('W_battery', 	'Weight|battery'),
					  ('W_propulsion', 	'Weight|propulsion'),
					  ('W_structure', 	'Weight|structure'),
					  ('W_equipment', 	'Weight|equipment')]

		W_residual_eqn = 'W_residual = (W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment)**2'
		self.add_subsystem('w_residual_comp',
							om.ExecComp(W_residual_eqn, units='kg'),
							promotes_inputs=input_list,
							promotes_outputs=[('W_residual','Weight|residual')])

		# If nonlinear solver to be used
		if sizing_mode:
			# This drives W_residual = 0 by varying W_total. LB and UB of W_total should be given.
			residual_balance = om.BalanceComp('W_total',
											   units='kg',
											   eq_units='kg',
											   lower=0.0,
											   upper=10000.0,
											   val=1500.0,
											   rhs_val=0.0,
											   use_mult=False)
			self.add_subsystem('weight_balance',
								residual_balance,
								promotes_inputs=[('lhs:W_total', 'Weight|residual')],
								promotes_outputs=[('W_total','Weight|takeoff')])

			# Add solvers for implicit relations
			self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=50, iprint=0, rtol=1e-3)
			self.nonlinear_solver.options['err_on_non_converge'] = False
			self.nonlinear_solver.options['reraise_child_analysiserror'] = True
			self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
			self.nonlinear_solver.linesearch.options['maxiter'] = 10
			self.nonlinear_solver.linesearch.options['iprint'] = 0
			self.linear_solver = om.DirectSolver(assemble_jac=True)
