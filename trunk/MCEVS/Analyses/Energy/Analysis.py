import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Power.Analysis import PowerRequirement
from MCEVS.Analyses.Geometry.Rotor import MeanChord
from MCEVS.Utils.Performance import record_performance_by_segments

class EnergyAnalysis(object):
	"""
	docstring for EnergyAnalysis
	"""
	def __init__(self, vehicle:object, mission:object, fidelity:dict):
		super(EnergyAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.fidelity = fidelity

	def evaluate(self, record=False):
		# print('### --- Solving for energy requirement --- ###')

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

		# MTOW should be defined since this is not in sizing mode
		if self.vehicle.weight.max_takeoff is None:
			raise ValueError('Vehicle MTOW should be defined, since PowerAnalysis() is never in sizing mode!')
		else:
			indeps.add_output('Weight|takeoff', self.vehicle.weight.max_takeoff, units='kg')

		# Hover climb RPM should be defined since this is not in sizing mode
		if self.vehicle.lift_rotor.RPM['hover_climb'] is None:
			raise ValueError('Hover climb RPM should be defined, since PowerAnalysis() is never in sizing mode!')
		else:
			indeps.add_output('LiftRotor|HoverClimb|RPM', self.vehicle.lift_rotor.RPM['hover_climb'], units='rpm')

		for segment in self.mission.segments:
			if segment.kind == 'HoverClimbConstantSpeed':
				indeps.add_output('Mission|hover_climb_speed', segment.speed, units='m/s')
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
			# Convert mean_c_to_R into mean_chord
			prob.model.add_subsystem('chord_calc_lift_rotor',
									  MeanChord(),
									  promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
									  promotes_outputs=[('mean_chord', 'LiftRotor|chord')])
			prob.model.add_subsystem('chord_calc_propeller',
									  MeanChord(),
									  promotes_inputs=[('mean_c_to_R', 'Propeller|mean_c_to_R'), ('R', 'Propeller|radius')],
									  promotes_outputs=[('mean_chord', 'Propeller|chord')])

		# Core energy module
		prob.model.add_subsystem('energy_model',
								  EnergyConsumption(mission=self.mission,
								  				   vehicle=self.vehicle,
								  				   fidelity=self.fidelity),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		# Run the model (not in sizing mode !!!)
		prob.setup(check=False)
		prob.run_model()

		if record:
			record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

		return prob

class EnergyConsumption(om.Group):
	"""
	Computes the energy consumption of an eVTOL given the vehicle specifications and mission requirements.
	It also computes disk loading in hover and cruise.
	Inputs:
	(mission requirements)
		Mission object 			: an object containing mission profile information (including gravity and atmoshphere constant objects)
	(vehicle definition)
		Vehicle object 			: an object that defines the vehicle
	(eVTOL design variables)
		Weight|takeoff 			: total take-off weight [kg]
		Mission|cruise_speed 	: cruising speed of the eVTOL [m/s]
		LiftRotor|radius		: lifting rotor radius [m]
		Propeller|radius		: cruising rotor radius [m] 		(for lift+cruise only)
		Wing|area 				: wing area [m**2]					(for lift+cruise only)
		Wing|aspect_ratio		: wing aspect ratio 				(for lift+cruise only)
		Rotor|advance_ratio 	: rotor advance ratio				(for multirotor only)
		Propeller|advance_ratio	: propeller advance ratio			(for lift+cruise only)
	Outputs:
	(major performances)
		Energy|entire_mission
	"""
	def initialize(self):
		self.options.declare('mission', types=object, desc='Mission object')
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

	def setup(self):

		# Unpacking option objects
		mission 	 = self.options['mission']
		vehicle 	 = self.options['vehicle']
		fidelity 	 = self.options['fidelity']

		# -------------------------------------------------------------#
		# --- Calculate power consumptions for each flight segment --- #
		# -------------------------------------------------------------#

		self.add_subsystem('power_requirement',
							PowerRequirement(mission=mission,
											 vehicle=vehicle,
											 fidelity=fidelity),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# Mission variables
		indep = self.add_subsystem('mission_var', om.IndepVarComp())
		for id in range(1,mission.n_segments+1):
			indep.add_output(f'segment_time_{id}', val=mission.segments[id-1].duration, units='s')
		indep.add_output('n_repetition', val=mission.n_repetition)

		# Writing energy equation from power
		# energy_cnsmp = power_segment_1 * segment_time_1 + ... + power_segment_n * segment_time_n
		energy_eq 	 = 'energy_cnsmp = '
		kwargs_power = {}
		kwargs_time  = {}
		for i in range(1,mission.n_segments+1):
			if i == mission.n_segments:
				energy_eq += f'power_segment_{i} * segment_time_{i}'
			else:
				energy_eq += f'power_segment_{i} * segment_time_{i} + '
			kwargs_power[f'power_segment_{i}'] = {'units': 'W'}
			kwargs_time[f'segment_time_{i}'] = {'units': 's'}

		# ------------------------------------------------------------#
		# --- Calculate energy consumptions for the whole mission --- #
		# ------------------------------------------------------------#

		# -- One mission energy --- #
		energy_comp_one = om.ExecComp(energy_eq,
									  energy_cnsmp={'units': 'W * s'},
									  **kwargs_power, **kwargs_time)

		self.add_subsystem('energy_one_mission', energy_comp_one, promotes_outputs=[('energy_cnsmp','Energy|one_mission')])

		for i in range(1,mission.n_segments+1): 
			self.connect(f'Power|segment_{i}', f'energy_one_mission.power_segment_{i}')
			self.connect(f'mission_var.segment_time_{i}', f'energy_one_mission.segment_time_{i}')

		# -- Reserve mission energy --- #
		if len(mission.segments) == mission.n_segments: # meaning that there is no reserve mission
			indep.add_output('Energy|reserve_mission', val=0.0, units='W * s')

		else:
			indep.add_output('reserve_segment_time', val=mission.reserve_mission_duration, units='s')
			energy_comp_reserve = om.ExecComp('reserve_energy = reserve_power * reserve_segment_time',
											   reserve_energy={'units':'W*s'}, reserve_power={'units':'W'}, reserve_segment_time={'units':'s'})

			self.add_subsystem('reserve_energy', energy_comp_reserve,
								promotes_inputs=[('reserve_power','Power|reserve_segment'),('reserve_segment_time','mission_var.reserve_segment_time')],
								promotes_outputs=[('reserve_energy','Energy|reserve_mission')])

		# -- Entire mission energy --- #
		energy_comp_total = om.ExecComp('total_energy = n * energy_one_mission + reserve_energy',
										 total_energy={'units': 'W * s'}, energy_one_mission={'units': 'W * s'}, reserve_energy={'units': 'W * s'})

		self.add_subsystem('total_energy', energy_comp_total,
							promotes_inputs=[('n','mission_var.n_repetition'),('energy_one_mission','Energy|one_mission'),('reserve_energy','Energy|reserve_mission')],
							promotes_outputs=[('total_energy','Energy|entire_mission')])
