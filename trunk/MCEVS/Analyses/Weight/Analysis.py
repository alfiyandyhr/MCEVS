import numpy as np
import openmdao.api as om
import os
import sys
import copy

from MCEVS.Analyses.Energy.Analysis import EnergyConsumption
from MCEVS.Analyses.Weight.Battery.Groups import BatteryWeight
from MCEVS.Analyses.Weight.Propulsion.Groups import PropulsionWeight
from MCEVS.Analyses.Weight.Structure.Groups import StructureWeight
from MCEVS.Analyses.Weight.Equipment.Groups import EquipmentWeight
from MCEVS.Analyses.Geometry.Rotor import MeanChord
from MCEVS.Analyses.Geometry.Clearance import LiftRotorClearanceConstraint

from MCEVS.Utils.Performance import record_performance_by_segments
from MCEVS.Utils.Checks import check_fidelity_dict


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
		self.max_takeoff = mtow	 # defined, or sized
		self.gross_takeoff = None
		self.payload = None
		self.battery = None
		self.propulsion = None
		self.structure = None
		self.equipment = None
		self.is_sized = False

	def print_info(self):
		print(f'Weight|maximum_takeoff= {self.max_takeoff} kg')
		print(f'Weight|gross_takeoff= {self.gross_takeoff} kg')
		print(f'Weight|payload= {self.payload} kg')
		print(f'Weight|battery= {self.battery} kg')
		print(f'Weight|propulsion= {self.propulsion} kg')
		print(f'Weight|structure= {self.structure} kg')
		print(f'Weight|equipment= {self.equipment} kg')


class WeightAnalysis(object):
	"""
	docstring for WeightAnalysis
	"""
	def __init__(self, vehicle: object, mission: object, fidelity: dict, weight_type=None, sizing_mode=True, solved_by='optimization'):
		super(WeightAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.fidelity = fidelity

		# Check solver fidelity
		check_fidelity_dict(self.fidelity, self.vehicle.configuration)

		self.sizing_mode = sizing_mode
		self.solved_by = solved_by
		self.weight_type = weight_type  # ['maximum', 'gross']
		if self.solved_by not in ['optimization', 'nonlinear_solver']:
			raise NotImplementedError('"solved_by" should be either "optimization" or "nonlinear_solver"')

	def evaluate(self, record=False, weight_guess=None, print=True):

		if self.weight_type not in ['maximum', 'gross']:
			raise ValueError('"weight_type" should be either "maximum" or "gross"')

		if not print: 
			if os.name == 'posix':
				sys.stdout = open('/dev/null', 'w')  # Redirect stdout to /dev/null
			if os.name == 'nt':
				sys.stdout = open(os.devnull, 'w')  # Redirect stdout to os.devnull

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 				= self.vehicle.lift_rotor.radius 					# m
			r_hub_lift_rotor 			= self.vehicle.lift_rotor.hub_radius 				# m
			mean_c_to_R_lift_rotor 		= self.vehicle.lift_rotor.mean_c_to_R
			global_twist_lift_rotor 	= self.vehicle.lift_rotor.global_twist  			# deg

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 				= self.vehicle.lift_rotor.radius 					# m
			r_hub_lift_rotor 			= self.vehicle.lift_rotor.hub_radius 				# m
			mean_c_to_R_lift_rotor 		= self.vehicle.lift_rotor.mean_c_to_R
			global_twist_lift_rotor 	= self.vehicle.lift_rotor.global_twist  			# deg
			r_propeller 				= self.vehicle.propeller.radius 					# m
			mean_c_to_R_propeller 		= self.vehicle.propeller.mean_c_to_R
			wing_area 					= self.vehicle.wing.area 							# m**2
			wing_aspect_ratio 			= self.vehicle.wing.aspect_ratio
			wing_airfoil_CL_alpha 		= self.vehicle.wing.airfoil.CL_alpha 				# 1/rad
			wing_airfoil_CL_0 			= self.vehicle.wing.airfoil.CL_0
			htail_area					= self.vehicle.horizontal_tail.area 				# m**2
			htail_aspect_ratio 			= self.vehicle.horizontal_tail.aspect_ratio
			htail_max_root_thickness 	= self.vehicle.horizontal_tail.max_root_thickness 	# m
			vtail_area					= self.vehicle.vertical_tail.area 					# m**2
			vtail_aspect_ratio 			= self.vehicle.vertical_tail.aspect_ratio
			vtail_max_root_thickness 	= self.vehicle.vertical_tail.max_root_thickness 	# m
			vtail_sweep_angle 			= self.vehicle.vertical_tail.sweep_angle 			# deg
			l_fuse 						= self.vehicle.fuselage.length 						# m

		# --- OpenMDAO probolem --- #
		prob = om.Problem(reports=False)
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		for segment in self.mission.segments:
			if segment.kind not in ['ConstantPower', 'NoCreditClimb', 'NoCreditDescent', 'ReserveCruise']:
				indeps.add_output(f'Mission|segment_{segment.id}|speed', segment.speed, units='m/s')
				indeps.add_output(f'Mission|segment_{segment.id}|distance', segment.distance, units='m')
			if segment.kind == 'HoverClimbConstantSpeed':
				indeps.add_output('LiftRotor|HoverClimb|RPM', self.vehicle.lift_rotor.RPM['hover_climb'], units='rpm')
			if segment.kind == 'CruiseConstantSpeed':
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
			indeps.add_output('Wing|airfoil|CL_alpha', wing_airfoil_CL_alpha, units='1/rad')
			indeps.add_output('Wing|airfoil|CL_0', wing_airfoil_CL_0)
			indeps.add_output('HorizontalTail|area', htail_area, units='m**2')
			indeps.add_output('HorizontalTail|aspect_ratio', htail_aspect_ratio)
			indeps.add_output('HorizontalTail|max_root_thickness', htail_max_root_thickness, units='m')
			indeps.add_output('VerticalTail|area', vtail_area, units='m**2')
			indeps.add_output('VerticalTail|aspect_ratio', vtail_aspect_ratio)
			indeps.add_output('VerticalTail|max_root_thickness', vtail_max_root_thickness, units='m')
			indeps.add_output('VerticalTail|sweep_angle', vtail_sweep_angle, units='deg')
			indeps.add_output('Fuselage|length', l_fuse, units='m')

		# Variables needed for BEMT
		if self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
			n_sections = self.vehicle.lift_rotor.n_section
			r_to_R_list = self.vehicle.lift_rotor.r_to_R_list
			c_to_R_list = self.vehicle.lift_rotor.c_to_R_list
			w_to_R_list = self.vehicle.lift_rotor.w_to_R_list
			if self.vehicle.lift_rotor.pitch_linear_grad is not None:
				indeps.add_output('LiftRotor|pitch_linear_grad', self.vehicle.lift_rotor.pitch_linear_grad, units='deg')
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

		# --- Core weight module --- #

		# Maximum takeoff weight
		if self.weight_type == 'maximum':
			prob.model.add_subsystem('mtow_model',
									  MTOWEstimation(mission=self.mission,
									  				 vehicle=self.vehicle,
									  				 fidelity=self.fidelity,
									  				 sizing_mode=False if self.solved_by == 'optimization' else self.sizing_mode,
									  				 rhs_checking=False),
									  promotes_inputs=['*'],
									  promotes_outputs=['*'])
			# Sizing or not sizing
			if not self.sizing_mode:
				weight_guess = 1500.0 if weight_guess is None else weight_guess
				indeps.add_output('Weight|takeoff', weight_guess, units='kg')  # mtow initial guess
				prob.setup(check=False)
				# prob.check_partials(compact_print=True, method='fd')
				prob.run_model()

			else:
				if self.solved_by == 'optimization':
					prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
					weight_guess = 1500.0 if weight_guess is None else weight_guess
					indeps.add_output('Weight|takeoff', weight_guess, units='kg')  # mtow initial guess

					if self.fidelity['power_model']['hover_climb'] in ['MomentumTheory', 'ModifiedMomentumTheory']:
						prob.model.add_design_var('Weight|takeoff', lower=600, upper=10000, units='kg')
						prob.model.add_objective('Weight|residual', units='kg')
						prob.setup(check=False)
						prob.run_driver()

					elif self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
						prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10.0, upper=5000.0, units='rpm')
						prob.model.add_design_var('Weight|takeoff', lower=600.0, upper=10000.0, units='kg')
						prob.model.add_design_var('LiftRotor|global_twist', lower=0.0, upper=100.0, units='deg')
						prob.model.add_objective('Weight|residual', units='kg')
						prob.model.add_constraint('LiftRotor|HoverClimb|thrust_residual_square', lower=0, upper=0.1, units=None)
						prob.setup(check=False)
						prob.run_driver()

				elif self.solved_by == 'nonlinear_solver':

					if self.fidelity['power_model']['hover_climb'] == 'MomentumTheory':
						prob.setup(check=False)
						prob.run_model()

					elif self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
						prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
						prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10.0, upper=5000.0, units='rpm')
						prob.model.add_objective('LiftRotor|HoverClimb|thrust_residual_square', units=None)
						prob.setup(check=False)
						prob.run_driver()

			# VehicleWeight() bookkeeping
			self.vehicle.weight.payload = prob.get_val('Weight|payload')
			self.vehicle.weight.max_takeoff = prob.get_val('Weight|takeoff')
			self.vehicle.weight.gross_takeoff = prob.get_val('Weight|takeoff')
			self.vehicle.weight.battery = prob.get_val('Weight|battery')
			self.vehicle.weight.propulsion = prob.get_val('Weight|propulsion')
			self.vehicle.weight.structure = prob.get_val('Weight|structure')
			self.vehicle.weight.equipment = prob.get_val('Weight|equipment')
			self.vehicle.weight.is_sized = True

		# Gross takeoff weight				
		elif self.weight_type == 'gross':

			if not self.vehicle.weight.is_sized:
				raise ValueError('vehicle.weight.is_sized == False; vehicle should be sized first!')

			indeps.add_output('Weight|propulsion', self.vehicle.weight.propulsion, units='kg')
			indeps.add_output('Weight|structure', self.vehicle.weight.structure, units='kg')
			indeps.add_output('Weight|equipment', self.vehicle.weight.equipment, units='kg')

			prob.model.add_subsystem('gtow_model',
									  GTOWEstimation(mission=self.mission,
									  				 vehicle=self.vehicle,
									  				 fidelity=self.fidelity,
									  				 sizing_mode=False if self.solved_by == 'optimization' else self.sizing_mode,
									  				 rhs_checking=False),
									  promotes_inputs=['*'],
									  promotes_outputs=['*'])

			# Sizing or not sizing
			if not self.sizing_mode:
				indeps.add_output('Weight|takeoff', self.vehicle.weight.max_takeoff if weight_guess is None else weight_guess, units='kg')  # gtow initial guess
				prob.setup(check=False)
				# prob.check_partials(compact_print=True, method='fd')
				prob.run_model()

			else:
				if self.solved_by == 'optimization':
					prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
					indeps.add_output('Weight|takeoff', self.vehicle.weight.max_takeoff if weight_guess is None else weight_guess, units='kg')  # gtow initial guess

					if self.fidelity['power_model']['hover_climb'] in ['MomentumTheory', 'ModifiedMomentumTheory']:
						prob.model.add_design_var('Weight|takeoff', lower=600, upper=10000, units='kg')
						prob.model.add_objective('Weight|residual', units='kg')
						prob.setup(check=False)
						prob.run_driver()

					elif self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
						prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10.0, upper=5000.0, units='rpm')
						prob.model.add_design_var('Weight|takeoff', lower=600.0, upper=10000.0, units='kg')
						prob.model.add_design_var('LiftRotor|global_twist', lower=0.0, upper=100.0, units='deg')
						prob.model.add_objective('Weight|residual', units='kg')
						prob.model.add_constraint('LiftRotor|HoverClimb|thrust_residual_square', lower=0, upper=0.1, units=None)
						prob.setup(check=False)
						prob.run_driver()

				elif self.solved_by == 'nonlinear_solver':

					if self.fidelity['power_model']['hover_climb'] == 'MomentumTheory':
						prob.setup(check=False)
						prob.run_model()

					elif self.fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
						prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
						prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10.0, upper=5000.0, units='rpm')
						prob.model.add_objective('LiftRotor|HoverClimb|thrust_residual_square', units=None)
						prob.setup(check=False)
						prob.run_driver()

			# VehicleWeight() bookkeeping
			self.vehicle.weight.payload = prob.get_val('Weight|payload')
			self.vehicle.weight.gross_takeoff = prob.get_val('Weight|takeoff')
			self.vehicle.weight.battery = prob.get_val('Weight|battery')

		if record:
			record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

		# Reset stdout
		sys.stdout = sys.__stdout__  # Reset stdout back to the default

		return prob


class MTOWEstimation(om.Group):
	"""
	Computes eVTOL maximum takeoff weight estimation given the design variables and mission requirement.
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
		Weight|payload
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
		self.options.declare('rhs_checking', types=bool, desc='rhs_checking in OpenMDAO linear solver')

	def setup(self):
		
		# Unpacking option objects
		mission 	 	= self.options['mission']
		vehicle 	 	= self.options['vehicle']
		fidelity 	 	= self.options['fidelity']
		sizing_mode 	= self.options['sizing_mode']
		rhs_checking 	= self.options['rhs_checking']

		# Unpacking battery parameters
		battery_rho 			= vehicle.battery.density
		battery_eff 			= vehicle.battery.efficiency
		battery_max_discharge 	= vehicle.battery.max_discharge

		# --- Calculate total energy consumptions to fly the mission --- #
		self.add_subsystem('energy',
							EnergyConsumption(mission=mission,
											  vehicle=vehicle,
											  fidelity=fidelity,
											  rhs_checking=rhs_checking),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# --- Calculate weight estimation of each component --- #

		# 1. Battery weight
		# battery weight is computed taking into account loss in efficiency, avionics power, and its maximum discharge rate

		self.add_subsystem('battery_weight',
							BatteryWeight(battery_rho=battery_rho, battery_eff=battery_eff, battery_max_discharge=battery_max_discharge),
							promotes_inputs=[('required_energy', 'Energy|entire_mission')],
							promotes_outputs=['Weight|battery'])

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
		if fidelity['weight_model']['structure'] == 'Roskam':
			if vehicle.configuration == 'Multirotor':
				input_list_struct = ['Weight|takeoff']
			elif vehicle.configuration == 'LiftPlusCruise':
				input_list_struct = ['Weight|takeoff', 'Wing|*', 'HorizontalTail|*', 'VerticalTail|*']
		elif fidelity['weight_model']['structure'] == 'M4Regression':
			if vehicle.configuration == 'LiftPlusCruise':
				for segment in mission.segments:
					if segment.kind == 'CruiseConstantSpeed':
						cruise_segment_id = segment.id
				input_list_struct = ['Weight|takeoff', 'Weight|battery', 'Wing|*', 'HorizontalTail|*', 'VerticalTail|*', 'Fuselage|length', ('v_cruise', f'Mission|segment_{cruise_segment_id}|speed')]
		self.add_subsystem('structure_weight',
							StructureWeight(vehicle=vehicle, fidelity=fidelity, tf_structure=vehicle.tf_structure),
							promotes_inputs=input_list_struct,
							promotes_outputs=['Weight|*'])
		# includes mission segment power for takeoff for sizing booms if using Roskam method
		if fidelity['weight_model']['structure'] == 'Roskam':
			for i, segment in enumerate(mission.segments):
				if segment.kind in ['HoverClimbConstantSpeed']:
					self.connect(f'Power|LiftRotor|segment_{i+1}', 'structure_weight.total_req_takeoff_power')

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
		payload_weight = om.IndepVarComp('Weight|payload', val=n_pax * payload_per_pax, units='kg')
		self.add_subsystem('payload', payload_weight, promotes=['*'])

		# W_residual should then be driven to 0 by a nonlinear solver or treated as an optimization constraint
		input_list = [('W_total', 'Weight|takeoff'),
					  ('W_payload', 'Weight|payload'),
					  ('W_battery', 'Weight|battery'),
					  ('W_propulsion', 'Weight|propulsion'),
					  ('W_structure', 'Weight|structure'),
					  ('W_equipment', 'Weight|equipment')]

		W_residual_eqn = 'W_residual = (W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment)**2'
		self.add_subsystem('w_residual_comp',
							om.ExecComp(W_residual_eqn, units='kg'),
							promotes_inputs=input_list,
							promotes_outputs=[('W_residual', 'Weight|residual')])

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
								promotes_outputs=[('W_total', 'Weight|takeoff')])

			# Add solvers for implicit relations
			self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=50, iprint=0, rtol=1e-3)
			self.nonlinear_solver.options['err_on_non_converge'] = False
			self.nonlinear_solver.options['reraise_child_analysiserror'] = True
			self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
			self.nonlinear_solver.linesearch.options['maxiter'] = 10
			self.nonlinear_solver.linesearch.options['iprint'] = 0
			self.linear_solver = om.DirectSolver(assemble_jac=True)


class GTOWEstimation(om.Group):
	"""
	Computes eVTOL gross takeoff weight estimation given the design variables and mission requirement.
	Must be used with an optimizer (or a nonlinear solver) to converge the weight residual.
	"""

	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('sizing_mode', types=bool, desc='Whether to use in a sizing mode')
		self.options.declare('rhs_checking', types=bool, desc='rhs_checking in OpenMDAO linear solver')

	def setup(self):
		
		# Unpacking option objects
		mission 	 	= self.options['mission']
		vehicle 	 	= self.options['vehicle']
		fidelity 	 	= self.options['fidelity']
		sizing_mode 	= self.options['sizing_mode']
		rhs_checking 	= self.options['rhs_checking']

		# Unpacking battery parameters
		battery_rho 			= vehicle.battery.density
		battery_eff 			= vehicle.battery.efficiency
		battery_max_discharge 	= vehicle.battery.max_discharge

		# --- Calculate total energy consumptions to fly the mission --- #
		self.add_subsystem('energy',
							EnergyConsumption(mission=mission,
											  vehicle=vehicle,
											  fidelity=fidelity,
											  rhs_checking=rhs_checking),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# --- Calculate weight estimation of each component --- #

		# 1. Battery weight
		# battery weight is computed taking into account loss in efficiency, avionics power, and its maximum discharge rate
		self.add_subsystem('battery_weight',
							BatteryWeight(battery_rho=battery_rho, battery_eff=battery_eff, battery_max_discharge=battery_max_discharge),
							promotes_inputs=[('required_energy', 'Energy|entire_mission')],
							promotes_outputs=['Weight|battery'])

		# 2. Weight residuals
		# W_residual = (W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment)**2

		# Payload weight
		# 1 PAX = 100.0 kg (default)
		n_pax = vehicle.fuselage.number_of_passenger
		payload_per_pax = vehicle.fuselage.payload_per_pax
		payload_weight = om.IndepVarComp('Weight|payload', val=n_pax * payload_per_pax, units='kg')
		self.add_subsystem('payload', payload_weight, promotes=['*'])

		# W_residual should then be driven to 0 by a nonlinear solver or treated as an optimization constraint
		input_list = [('W_total', 'Weight|takeoff'),
					  ('W_payload', 'Weight|payload'),
					  ('W_battery', 'Weight|battery'),
					  ('W_propulsion', 'Weight|propulsion'),
					  ('W_structure', 'Weight|structure'),
					  ('W_equipment', 'Weight|equipment')]

		W_residual_eqn = 'W_residual = (W_total - W_payload - W_battery - W_propulsion - W_structure - W_equipment)**2'
		self.add_subsystem('w_residual_comp',
							om.ExecComp(W_residual_eqn, units='kg'),
							promotes_inputs=input_list,
							promotes_outputs=[('W_residual', 'Weight|residual')])

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
								promotes_outputs=[('W_total', 'Weight|takeoff')])

			# Add solvers for implicit relations
			self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True, maxiter=50, iprint=0, rtol=1e-3)
			self.nonlinear_solver.options['err_on_non_converge'] = False
			self.nonlinear_solver.options['reraise_child_analysiserror'] = True
			self.nonlinear_solver.linesearch = om.ArmijoGoldsteinLS()
			self.nonlinear_solver.linesearch.options['maxiter'] = 10
			self.nonlinear_solver.linesearch.options['iprint'] = 0
			self.linear_solver = om.DirectSolver(assemble_jac=True)


class MultiPointMTOWEstimation(om.Group):
	"""
	Computes multiple MTOWEstimation()
	"""

	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('multipoint_options', types=dict, desc='Multipoint options, see DesignProblem()')

	def setup(self):
		
		# Unpacking option objects
		mission 	 		= self.options['mission']
		vehicle 	 		= self.options['vehicle']
		fidelity 	 		= self.options['fidelity']
		multipoint_options 	= self.options['multipoint_options']

		# --- MTOWEstimation at multiple points based on the multipoint type --- #

		# Vehicle configuration
		output_list = []

		for n in range(1, multipoint_options['n_points'] + 1):
			if vehicle.configuration == 'Multirotor':
				output_list.append([('Energy|entire_mission', f'Point_{n}|Energy|entire_mission'),
									('Weight|residual', f'Point_{n}|Weight|residual'),
									('LiftRotor|Cruise|mu', f'Point_{n}|LiftRotor|Cruise|mu'),
									('LiftRotor|Cruise|thrust_coefficient', f'Point_{n}|LiftRotor|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'Point_{n}|LiftRotor|HoverClimb|T_to_P'),
									('LiftRotor|Cruise|T_to_P', f'Point_{n}|LiftRotor|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'Point_{n}|LiftRotor|HoverDescent|T_to_P')])

			elif vehicle.configuration == 'LiftPlusCruise':
				output_list.append([('Energy|entire_mission', f'Point_{n}|Energy|entire_mission'),
									('Weight|residual', f'Point_{n}|Weight|residual'),
									('Aero|Cruise|CL', f'Point_{n}|Aero|Cruise|CL'),
									('Propeller|Cruise|J', f'Point_{n}|Propeller|Cruise|J'),
									('Propeller|Cruise|thrust_coefficient', f'Point_{n}|Propeller|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'Point_{n}|LiftRotor|HoverClimb|T_to_P'),
									('Propeller|Cruise|T_to_P', f'Point_{n}|Propeller|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'Point_{n}|LiftRotor|HoverDescent|T_to_P')])

		# Battery energy density
		if multipoint_options['type'] == 'battery_energy_density':

			vehicles = []

			for n in range(1, multipoint_options['n_points'] + 1):

				if n == 1:
					vehicles.append(vehicle)
				else:
					vehicles.append(copy.deepcopy(vehicle))
				vehicles[n - 1].battery.density = multipoint_options['value_list'][n - 1]

				self.add_subsystem(f'point_{n}_analysis',
									MTOWEstimation(mission=mission,
								  				   vehicle=vehicles[n - 1],
								  				   fidelity=fidelity,
								  				   sizing_mode=False,
								  				   rhs_checking=True),
									promotes_inputs=[('Weight|takeoff', f'Point_{n}|Weight|takeoff'), '*'],
									promotes_outputs=output_list[n - 1])			

		# Weighted sum of takeoff weight or energy
		# weighted_sum_of_metric = coeff_1 * metric_1 + ... + coeff_n * metric_n
		if multipoint_options['objective'] in ['weighted_sum_of_takeoff_weight', 'weighted_sum_of_energy']:
			metric_eq = f"{multipoint_options['objective']} = "
			kwargs_coeff = {}
			kwargs_metric = {f"{multipoint_options['objective']}": {'units': 'kg' if multipoint_options['objective'] == 'weighted_sum_of_takeoff_weight' else 'W*h'}}
			input_list = []
			indep = self.add_subsystem('multipoint_coeffs', om.IndepVarComp())
			for n in range(1, multipoint_options['n_points'] + 1):
				indep.add_output(f'coeff_{n}', val=multipoint_options['weight_coeffs'][n - 1], units=None)
				if n == multipoint_options['n_points']:
					metric_eq += f'coeff_{n} * metric_{n}'
				else:
					metric_eq += f'coeff_{n} * metric_{n} + '
				kwargs_coeff[f'coeff_{n}'] = {'units': None}
				if multipoint_options['objective'] == 'weighted_sum_of_takeoff_weight':
					kwargs_metric[f'metric_{n}'] = {'units': 'kg'}
					input_list.append((f'metric_{n}', f'Point_{n}|Weight|takeoff'))
				elif multipoint_options['objective'] == 'weighted_sum_of_energy':
					kwargs_metric[f'metric_{n}'] = {'units': 'kW*h'}
					input_list.append((f'metric_{n}', f'Point_{n}|Energy|entire_mission'))

			# Objective evaluation
			self.add_subsystem('multipoint_single_obj',
								om.ExecComp(metric_eq, **kwargs_coeff, **kwargs_metric),
								promotes_inputs=input_list,
								promotes_outputs=[f"{multipoint_options['objective']}"])
			
			for n in range(1, multipoint_options['n_points'] + 1):
				self.connect(f'multipoint_coeffs.coeff_{n}', f'multipoint_single_obj.coeff_{n}')


class MultiPointMTOWEstimationWithFixedEmptyWeight(om.Group):
	"""
	Computes one MTOWEstimation() and multiple GTOWEstimation()
	"""

	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('multipoint_options', types=dict, desc='Multipoint options, see DesignProblem()')

	def setup(self):
		
		# Unpacking option objects
		mission 	 		= self.options['mission']
		vehicle 	 		= self.options['vehicle']
		fidelity 	 		= self.options['fidelity']
		multipoint_options 	= self.options['multipoint_options']

		# --- GTOWEstimation at multiple points based on the multipoint type --- #

		# Vehicle configuration
		output_list = []

		for n in range(1, multipoint_options['n_points'] + 1):
			if vehicle.configuration == 'Multirotor':
				output_list.append([('Energy|entire_mission', f'Point_{n}|Energy|entire_mission'),
									('Weight|residual', f'Point_{n}|Weight|residual'),
									('LiftRotor|Cruise|mu', f'Point_{n}|LiftRotor|Cruise|mu'),
									('LiftRotor|Cruise|thrust_coefficient', f'Point_{n}|LiftRotor|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'Point_{n}|LiftRotor|HoverClimb|T_to_P'),
									('LiftRotor|Cruise|T_to_P', f'Point_{n}|LiftRotor|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'Point_{n}|LiftRotor|HoverDescent|T_to_P')])

			elif vehicle.configuration == 'LiftPlusCruise':
				output_list.append([('Energy|entire_mission', f'Point_{n}|Energy|entire_mission'),
									('Weight|residual', f'Point_{n}|Weight|residual'),
									('Aero|Cruise|CL', f'Point_{n}|Aero|Cruise|CL'),
									('Propeller|Cruise|J', f'Point_{n}|Propeller|Cruise|J'),
									('Propeller|Cruise|thrust_coefficient', f'Point_{n}|Propeller|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'Point_{n}|LiftRotor|HoverClimb|T_to_P'),
									('Propeller|Cruise|T_to_P', f'Point_{n}|Propeller|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'Point_{n}|LiftRotor|HoverDescent|T_to_P')])

		# Battery energy density
		if multipoint_options['type'] == 'battery_energy_density':

			vehicles = []

			for n in range(1, multipoint_options['n_points'] + 1):

				if n == 1:
					vehicles.append(vehicle)
				else:
					vehicles.append(copy.deepcopy(vehicle))
				vehicles[n - 1].battery.density = multipoint_options['value_list'][n - 1]

				if n == 1:
					output_list[0].append(('Weight|propulsion', 'Point_1|Weight|propulsion'))
					output_list[0].append(('Weight|structure', 'Point_1|Weight|structure'))
					output_list[0].append(('Weight|equipment', 'Point_1|Weight|equipment'))
					self.add_subsystem(f'point_{n}_analysis',
										MTOWEstimation(mission=mission,
									  				   vehicle=vehicles[n - 1],
									  				   fidelity=fidelity,
									  				   sizing_mode=False,
									  				   rhs_checking=True),
										promotes_inputs=[('Weight|takeoff', f'Point_{n}|Weight|takeoff'), '*'],
										promotes_outputs=output_list[n - 1])

				else:
					suboptimal_input = [('Weight|takeoff', f'Point_{n}|Weight|takeoff'),
										('Weight|propulsion', 'Point_1|Weight|propulsion'),
										('Weight|structure', 'Point_1|Weight|structure'),
										('Weight|equipment', 'Point_1|Weight|equipment'), '*']
					self.add_subsystem(f'point_{n}_analysis',
										GTOWEstimation(mission=mission,
									  				   vehicle=vehicles[n - 1],
									  				   fidelity=fidelity,
									  				   sizing_mode=False,
									  				   rhs_checking=True),
										promotes_inputs=suboptimal_input,
										promotes_outputs=output_list[n - 1])		

		# Weighted sum of takeoff weight or energy
		# weighted_sum_of_metric = coeff_1 * metric_1 + ... + coeff_n * metric_n
		if multipoint_options['objective'] in ['weighted_sum_of_takeoff_weight', 'weighted_sum_of_energy']:
			metric_eq = f"{multipoint_options['objective']} = "
			kwargs_coeff = {}
			kwargs_metric = {f"{multipoint_options['objective']}": {'units': 'kg' if multipoint_options['objective'] == 'weighted_sum_of_takeoff_weight' else 'W*h'}}
			input_list = []
			indep = self.add_subsystem('multipoint_coeffs', om.IndepVarComp())
			for n in range(1, multipoint_options['n_points'] + 1):
				indep.add_output(f'coeff_{n}', val=multipoint_options['weight_coeffs'][n - 1], units=None)
				if n == multipoint_options['n_points']:
					metric_eq += f'coeff_{n} * metric_{n}'
				else:
					metric_eq += f'coeff_{n} * metric_{n} + '
				kwargs_coeff[f'coeff_{n}'] = {'units': None}
				if multipoint_options['objective'] == 'weighted_sum_of_takeoff_weight':
					kwargs_metric[f'metric_{n}'] = {'units': 'kg'}
					input_list.append((f'metric_{n}', f'Point_{n}|Weight|takeoff'))
				elif multipoint_options['objective'] == 'weighted_sum_of_energy':
					kwargs_metric[f'metric_{n}'] = {'units': 'kW*h'}
					input_list.append((f'metric_{n}', f'Point_{n}|Energy|entire_mission'))

			# Objective evaluation
			self.add_subsystem('multipoint_single_obj',
								om.ExecComp(metric_eq, **kwargs_coeff, **kwargs_metric),
								promotes_inputs=input_list,
								promotes_outputs=[f"{multipoint_options['objective']}"])
			
			for n in range(1, multipoint_options['n_points'] + 1):
				self.connect(f'multipoint_coeffs.coeff_{n}', f'multipoint_single_obj.coeff_{n}')


class OffDesignMTOWEstimation(om.Group):
	"""
	Computes one MTOWEstimation() and one off-design GTOWEstimation()
	"""

	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('offdesign_options', types=dict, desc='Off-design options, see DesignProblem()')

	def setup(self):
		
		# Unpacking option objects
		mission 	 		= self.options['mission']
		vehicle 	 		= self.options['vehicle']
		fidelity 	 		= self.options['fidelity']
		offdesign_options 	= self.options['offdesign_options']

		# --- GTOWEstimation at one off-design point --- #

		points = ['OnDesign', 'OffDesign']
		output_list = []

		for point in points:

			if vehicle.configuration == 'Multirotor':
				output_list.append([('Energy|entire_mission', f'{point}|Energy|entire_mission'),
									('Weight|residual', f'{point}|Weight|residual'),
									('Weight|battery', f'{point}|Weight|battery'),
									('LiftRotor|Cruise|mu', f'{point}|LiftRotor|Cruise|mu'),
									('LiftRotor|Cruise|thrust_coefficient', f'{point}|LiftRotor|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'{point}|LiftRotor|HoverClimb|T_to_P'),
									('LiftRotor|Cruise|T_to_P', f'{point}|LiftRotor|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'{point}|LiftRotor|HoverDescent|T_to_P')])

			elif vehicle.configuration == 'LiftPlusCruise':
				output_list.append([('Energy|entire_mission', f'{point}|Energy|entire_mission'),
									('Weight|residual', f'{point}|Weight|residual'),
									('Weight|battery', f'{point}|Weight|battery'),
									('Aero|Cruise|CL', f'{point}|Aero|Cruise|CL'),
									('Propeller|Cruise|J', f'{point}|Propeller|Cruise|J'),
									('Propeller|Cruise|thrust_coefficient', f'{point}|Propeller|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'{point}|LiftRotor|HoverClimb|T_to_P'),
									('Propeller|Cruise|T_to_P', f'{point}|Propeller|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'{point}|LiftRotor|HoverDescent|T_to_P')])

		# Battery energy density
		if offdesign_options['type'] == 'battery_energy_density':

			vehicles = [vehicle, copy.deepcopy(vehicle)]
			vehicles[0].battery.density = offdesign_options['empty_weight_sized_at']
			vehicles[1].battery.density = offdesign_options['off_design_at']

			# On-design analysis (where empty weight is sized)
			output_list[0].append(('Weight|propulsion', 'OnDesign|Weight|propulsion'))
			output_list[0].append(('Weight|structure', 'OnDesign|Weight|structure'))
			output_list[0].append(('Weight|equipment', 'OnDesign|Weight|equipment'))
			self.add_subsystem('ondesign_analysis',
								MTOWEstimation(mission=mission,
							  				   vehicle=vehicles[0],
							  				   fidelity=fidelity,
							  				   sizing_mode=False,
							  				   rhs_checking=True),
								promotes_inputs=[('Weight|takeoff', 'OnDesign|Weight|takeoff'), '*'],
								promotes_outputs=output_list[0])

			# Off-design analysis (where objective function is evaluated)
			ondesign_input = [('Weight|takeoff', 'OffDesign|Weight|takeoff'),  # W_takeoff is suited to off-design
							  ('Weight|propulsion', 'OnDesign|Weight|propulsion'),
							  ('Weight|structure', 'OnDesign|Weight|structure'),
							  ('Weight|equipment', 'OnDesign|Weight|equipment'), '*']
			self.add_subsystem('offdesign_analysis',
								GTOWEstimation(mission=mission,
							  				   vehicle=vehicles[1],
							  				   fidelity=fidelity,
							  				   sizing_mode=False,
							  				   rhs_checking=True),
								promotes_inputs=ondesign_input,
								promotes_outputs=output_list[1])


class OffDesignMTOWEstimation2(om.Group):
	"""
	Computes one MTOWEstimation() and one off-design GTOWEstimation()
	"""

	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')
		self.options.declare('offdesign_options', types=dict, desc='Off-design options, see DesignProblem()')

	def setup(self):
		
		# Unpacking option objects
		mission 	 		= self.options['mission']
		vehicle 	 		= self.options['vehicle']
		fidelity 	 		= self.options['fidelity']
		offdesign_options 	= self.options['offdesign_options']

		# --- GTOWEstimation at one off-design point --- #

		points = ['OnDesign', 'OffDesign']
		output_list = []

		for point in points:

			if vehicle.configuration == 'Multirotor':
				output_list.append([('Energy|entire_mission', f'{point}|Energy|entire_mission'),
									('Weight|residual', f'{point}|Weight|residual'),
									('LiftRotor|Cruise|mu', f'{point}|LiftRotor|Cruise|mu'),
									('LiftRotor|Cruise|thrust_coefficient', f'{point}|LiftRotor|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'{point}|LiftRotor|HoverClimb|T_to_P'),
									('LiftRotor|Cruise|T_to_P', f'{point}|LiftRotor|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'{point}|LiftRotor|HoverDescent|T_to_P')])

			elif vehicle.configuration == 'LiftPlusCruise':
				output_list.append([('Energy|entire_mission', f'{point}|Energy|entire_mission'),
									('Weight|residual', f'{point}|Weight|residual'),
									('Aero|Cruise|CL', f'{point}|Aero|Cruise|CL'),
									('Propeller|Cruise|J', f'{point}|Propeller|Cruise|J'),
									('Propeller|Cruise|thrust_coefficient', f'{point}|Propeller|Cruise|thrust_coefficient'),
									('LiftRotor|HoverClimb|T_to_P', f'{point}|LiftRotor|HoverClimb|T_to_P'),
									('Propeller|Cruise|T_to_P', f'{point}|Propeller|Cruise|T_to_P'),
									('LiftRotor|HoverDescent|T_to_P', f'{point}|LiftRotor|HoverDescent|T_to_P')])

		# Battery energy density
		if offdesign_options['type'] == 'battery_energy_density':

			vehicles = [vehicle, copy.deepcopy(vehicle)]
			vehicles[0].battery.density = offdesign_options['empty_weight_sized_at']
			vehicles[1].battery.density = offdesign_options['off_design_at']

			# On-design analysis (where empty weight is sized)
			output_list[0].append(('Weight|propulsion', 'OnDesign|Weight|propulsion'))
			output_list[0].append(('Weight|structure', 'OnDesign|Weight|structure'))
			output_list[0].append(('Weight|equipment', 'OnDesign|Weight|equipment'))
			self.add_subsystem('ondesign_analysis',
								MTOWEstimation(mission=mission,
							  				   vehicle=vehicles[0],
							  				   fidelity=fidelity,
							  				   sizing_mode=False,
							  				   rhs_checking=True),
								promotes_inputs=[('Weight|takeoff', 'OnDesign|Weight|takeoff'), '*'],
								promotes_outputs=output_list[0])

			# Off-design analysis (where objective function is evaluated)
			output_list[1].append(('Weight|propulsion', 'OffDesign|Weight|propulsion'))
			output_list[1].append(('Weight|structure', 'OffDesign|Weight|structure'))
			output_list[1].append(('Weight|equipment', 'OffDesign|Weight|equipment'))
			self.add_subsystem('offdesign_analysis',
								MTOWEstimation(mission=mission,
							  				   vehicle=vehicles[1],
							  				   fidelity=fidelity,
							  				   sizing_mode=False,
							  				   rhs_checking=True),
								promotes_inputs=[('Weight|takeoff', 'OffDesign|Weight|takeoff'), '*'],
								promotes_outputs=output_list[1])

			# Constraint evaluation
			input_list = [('W_prop1', 'OnDesign|Weight|propulsion'), ('W_struct1', 'OnDesign|Weight|structure'), ('W_equip1', 'OnDesign|Weight|equipment'),
						  ('W_prop2', 'OffDesign|Weight|propulsion'), ('W_struct2', 'OffDesign|Weight|structure'), ('W_equip2', 'OffDesign|Weight|equipment')]
			self.add_subsystem('constraint_eval',
								om.ExecComp('W_empty_constr = (W_prop1 + W_struct1 + W_equip1 - W_prop2 - W_struct2 - W_equip2)**2',
											W_empty_constr={'units': 'kg**2'},
											W_prop1={'units': 'kg'}, W_struct1={'units': 'kg'}, W_equip1={'units': 'kg'},
											W_prop2={'units': 'kg'}, W_struct2={'units': 'kg'}, W_equip2={'units': 'kg'}),
								promotes_inputs=input_list,
								promotes_outputs=["W_empty_constr"])
