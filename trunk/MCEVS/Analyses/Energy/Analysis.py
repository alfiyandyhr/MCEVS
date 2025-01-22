import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Power.Analysis import PowerRequirement
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

		# MTOW should be defined if not in sizing mode
		mtow = self.vehicle.weight.max_takeoff 		# kg

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			c_lift_rotor 			= self.vehicle.lift_rotor.chord 		# m

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_hub_lift_rotor 		= self.vehicle.lift_rotor.hub_radius 	# m
			c_lift_rotor 			= self.vehicle.lift_rotor.chord 		# m
			r_propeller 			= self.vehicle.propeller.radius 		# m
			c_propeller 			= self.vehicle.propeller.chord  		# m
			wing_area 				= self.vehicle.wing.area 				# m**2
			wing_aspect_ratio 		= self.vehicle.wing.aspect_ratio

		for segment in self.mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				cruise_speed = segment.speed 			# m/s

		# --- OpenMDAO probolem --- #
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		indeps.add_output('Weight|takeoff', mtow, units='kg')

		for segment in self.mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				indeps.add_output('Mission|cruise_speed', segment.speed, units='m/s')
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Cruise|RPM', vehicle.lift_rotor.RPM['cruise'], units='RPM')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Cruise|RPM', vehicle.propeller.RPM['cruise'], units='RPM')
			if segment.kind == 'ClimbConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Climb|RPM', vehicle.lift_rotor.RPM['climb'], units='RPM')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Climb|RPM', vehicle.propeller.RPM['climb'], units='RPM')		
			if segment.kind == 'DescentConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					indeps.add_output('LiftRotor|Descent|RPM', vehicle.lift_rotor.RPM['descent'], units='RPM')
				elif self.vehicle.configuration == 'LiftPlusCruise':
					indeps.add_output('Propeller|Descent|RPM', vehicle.propeller.RPM['descent'], units='RPM')

		if self.vehicle.configuration == 'Multirotor':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|chord', c_lift_rotor, units='m')

		elif self.vehicle.configuration == 'LiftPlusCruise':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
			indeps.add_output('LiftRotor|chord', c_lift_rotor, units='m')
			indeps.add_output('Propeller|radius', r_propeller, units='m')
			indeps.add_output('Propeller|chord', c_propeller, units='m')
			indeps.add_output('Wing|area', wing_area, units='m**2')
			indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)

		# Lift rotor variables needed for BEMT
		if self.fidelity['hover_climb'] == 1:
			n_sections = self.vehicle.lift_rotor.n_section
			radius_list = np.array(self.vehicle.lift_rotor.r_to_R_list) * self.vehicle.lift_rotor.radius
			chord_list = np.array(self.vehicle.lift_rotor.c_to_R_list) * self.vehicle.lift_rotor.radius
			pitch_list = np.array(self.vehicle.lift_rotor.pitch_list)
			for i in range(n_sections):
				if i == 0:
					width = 2*(radius_list[i] - r_hub_lift_rotor)
				elif i == n_sections-1:
					width = 2*(r_lift_rotor - radius_list[i])
				else:
					width = radius_list[i] - radius_list[i-1]
				indeps.add_output(f'LiftRotor|Section{i+1}|radius', radius_list[i], units='m')
				indeps.add_output(f'LiftRotor|Section{i+1}|chord', chord_list[i], units='m')
				indeps.add_output(f'LiftRotor|Section{i+1}|pitch', pitch_list[i], units='deg')
				indeps.add_output(f'LiftRotor|Section{i+1}|width', width, units='m')
			indeps.add_output('LiftRotor|HoverClimb|RPM', 1000.0, units='rpm') # rpm guess
		
		prob.model.add_subsystem('energy_model',
								  EnergyConsumption(mission=self.mission,
								  					vehicle=self.vehicle,
								  					fidelity=self.fidelity),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		if self.fidelity['hover_climb'] == 0:
			prob.setup(check=False)
			prob.run_model()

		elif self.fidelity['hover_climb'] == 1:
			prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=False)
			prob.model.add_design_var('LiftRotor|HoverClimb|RPM', lower=10, upper=5000)
			prob.model.add_objective('LiftRotor|HoverClimb|thrust_residual_square')
			prob.setup(check=False)
			prob.run_driver()

		if record:
			record_performance_by_segments(prob, self.vehicle.configuration, self.mission)

		# self.total_energy_required = prob.get_val('energy_cnsmp', 'kW*h')[0]
		return prob
		# return f'Total energy required is {self.total_energy_required} kWh'

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
