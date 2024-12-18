import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Power.Analysis import PowerRequirement
from MCEVS.Utils.Performance import record_performance_by_segments

class EnergyAnalysis(object):
	"""
	docstring for EnergyAnalysis
	"""
	def __init__(self, vehicle:object, mission:object, constants:object, fidelity:dict):
		super(EnergyAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.constants = constants
		self.fidelity = fidelity

	def evaluate(self, record=False):
		# print('### --- Solving for energy requirement --- ###')

		# MTOW should be defined if not in sizing mode
		mtow = self.vehicle.weight.max_takeoff 		# kg

		# --- Design parameters --- #

		if self.vehicle.configuration == 'Multirotor':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			rotor_advance_ratio 	= self.vehicle.lift_rotor.advance_ratio

		elif self.vehicle.configuration == 'LiftPlusCruise':
			r_lift_rotor 			= self.vehicle.lift_rotor.radius 		# m
			r_propeller 			= self.vehicle.propeller.radius 		# m
			wing_area 				= self.vehicle.wing.area 				# m**2
			wing_aspect_ratio 		= self.vehicle.wing.aspect_ratio
			propeller_advance_ratio = self.vehicle.propeller.advance_ratio

		for segment in self.mission.segments:
			if segment.kind == 'CruiseConstantSpeed':
				cruise_speed = segment.speed 			# m/s

		# --- OpenMDAO probolem --- #
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

		indeps.add_output('Weight|takeoff', mtow, units='kg')
		indeps.add_output('Mission|cruise_speed', cruise_speed, units='m/s')

		if self.vehicle.configuration == 'Multirotor':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('LiftRotor|advance_ratio', rotor_advance_ratio)

		elif self.vehicle.configuration == 'LiftPlusCruise':
			indeps.add_output('LiftRotor|radius', r_lift_rotor, units='m')
			indeps.add_output('Propeller|radius', r_propeller, units='m')
			indeps.add_output('Wing|area', wing_area, units='m**2')
			indeps.add_output('Wing|aspect_ratio', wing_aspect_ratio)
			indeps.add_output('Propeller|advance_ratio', propeller_advance_ratio)
		
		prob.model.add_subsystem('energy_model',
								  EnergyConsumption(mission=self.mission,
								  					vehicle=self.vehicle,
								  					constants=self.constants,
								  					fidelity=self.fidelity),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()

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
		Mission object 			: an object containing mission profile information
	(vehicle definition)
		Vehicle object 			: an object that defines the vehicle
	(atmospheric condition)
		Constants object 		: an object containing constant values
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
		self.options.declare('constants', types=object, desc='Constants object')
		self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

	def setup(self):

		# Unpacking option objects
		mission 	= self.options['mission']
		vehicle 	= self.options['vehicle']
		constants 	= self.options['constants']
		fidelity 	= self.options['fidelity']

		# -------------------------------------------------------------#
		# --- Calculate power consumptions for each flight segment --- #
		# -------------------------------------------------------------#

		self.add_subsystem('power_requirement',
							PowerRequirement(mission=mission,
											 vehicle=vehicle,
											 constants=constants,
											 fidelity=fidelity),
							promotes_inputs=['*'],
							promotes_outputs=['*'])

		# Segment time
		indep = self.add_subsystem('segment_time', om.IndepVarComp())
		for id in range(1,len(mission.segments)+1):
			indep.add_output(f'segment_time_{id}', val=mission.segments[id-1].duration, units='s')

		# Writing energy equation from power
		# energy_cnsmp = power_segment_1 * segment_time_1 + ... + power_segment_n * segment_time_n
		energy_eq 	 = 'energy_cnsmp = '
		kwargs_power = {}
		kwargs_time  = {}
		for i in range(1,len(mission.segments)+1):
			if i == len(mission.segments):
				energy_eq += f'power_segment_{i} * segment_time_{i}'
			else:
				energy_eq += f'power_segment_{i} * segment_time_{i} + '
			kwargs_power[f'power_segment_{i}'] = {'units': 'W'}
			kwargs_time[f'segment_time_{i}'] = {'units': 's'}

		# ------------------------------------------------------------#
		# --- Calculate energy consumptions for the whole mission --- #
		# ------------------------------------------------------------#
		energy_comp = om.ExecComp(energy_eq,
								  energy_cnsmp={'units': 'W * s'},
								  **kwargs_power, **kwargs_time)

		self.add_subsystem('energy', energy_comp, promotes_outputs=[('energy_cnsmp','Energy|entire_mission')])

		for i in range(1,len(mission.segments)+1): 
			self.connect(f'Power|segment_{i}', f'energy.power_segment_{i}')
			self.connect(f'segment_time.segment_time_{i}', f'energy.segment_time_{i}')









