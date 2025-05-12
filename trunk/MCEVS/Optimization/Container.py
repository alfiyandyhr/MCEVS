import numpy as np
import openmdao.api as om
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL, StandardMultirotorEVTOL
from MCEVS.Optimization.Gradient_Based.Algorithm import run_gradient_based_optimization
from MCEVS.Optimization.Gradient_Free.Algorithm import run_gradient_free_optimization

class DesignProblem(object):
	"""
	docstring for DesignProblem
	This class object contains the problem definition of the eVTOL design

	minimize 		: objective functions
	with respect to : design variables
	subject to 		: inequality constraints, equality constraints
	under			: design conditions

	"""
	def __init__(self, vehicle:object, mission:object, fidelity:dict, kind:str, algorithm:'str'):
		super(DesignProblem, self).__init__()
		self.kind = kind # ['SingleObjectiveProblem', 'MultiObjectiveProblem', 'MultiPointSingleObjectiveProblem']

		# MCEVS core objects
		self.vehicle 	= vehicle
		self.mission 	= mission
		self.fidelity 	= fidelity

		# Design problem
		self.objectives				= {}
		self.design_variables		= {}
		self.constraints 			= {}
		self.default_input_values 	= {}

		# The driver
		self.algorithm				= algorithm

		# Initial, final, and optimal designs
		self.initial_design			= None
		self.final_design 			= None
		self.optimal_design			= None

		# Multipoint options
		if self.kind == 'MultiPointSingleObjectiveProblem':
			self.multipoint_options = {'type':str, 'n_points': int, 'value_list':list, 'objective':str, 'weight_coeffs': list}

	def run_optimization(self):
		"""
		Run the optimization and actually solve the problem
		"""

		# Initialize default input values
		self._initialize_default_input_values()

		if self.algorithm == 'gradient-based':
			# Run !!!
			result = run_gradient_based_optimization(self)

		elif self.algorithm == 'gradient-free':
			# Run !!!
			result = run_gradient_free_optimization(self)

		return result

	def add_objective(self, name:str, ref:float, units=None):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					- Mission|segment_{id}|speed
					- Wing|area
						  |aspect_ratio
					- LiftRotor|radius
							   |advance_ratio
					- Propeller|radius
							   |advance_ratio
		"""
		self.objectives[name] = [name, ref, units]

	def add_design_var(self, name:str, lower:float, upper:float, init:float, units=None):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					* Operation variables
						- Mission|segment_{id}|speed
						- LiftRotor|HoverClimb|RPM
								   |Climb|RPM
								   |Cruise|RPM
								   |Descent|RPM

					* Design variables
						- Weight|takeoff
						- Wing|area
							  |aspect_ratio
						- LiftRotor|radius
								   |advance_ratio
						- Propeller|radius
								   |advance_ratio
			lower: lower bound
			upper: upper bound
			init: initial value
			units: units
		"""
		# Append design variable info
		self.design_variables[name] = [lower, upper, init, units]

	def add_constraint(self, name, lower:float, upper:float, ref:float, units=None):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					- Mission|segment_{id}|speed
					- Wing|area
						  |aspect_ratio
					- LiftRotor|radius
							   |advance_ratio
					- Propeller|radius
							   |advance_ratio
			lower: lower bound
			upper: upper bound
			ref: reference value
			units: units
		"""
		self.constraints[name] = [lower, upper, ref, units]

	def _initialize_default_input_values(self):
		"""
		Parsing default input values that are not related to design variables but treated as input variables
		"""

		# Operation default input values
		for segment in self.mission.segments:
			if segment.kind not in ['ConstantPower','NoCreditClimb','NoCreditDescent','ReserveCruise']:
				self.default_input_values[f'Mission|segment_{segment.id}|speed'] = [segment.speed, 'm/s']
				self.default_input_values[f'Mission|segment_{segment.id}|distance'] = [segment.distance, 'm']
			if segment.kind == 'HoverClimbConstantSpeed':
				self.default_input_values['LiftRotor|HoverClimb|RPM'] = [self.vehicle.lift_rotor.RPM['hover_climb'], 'rpm']
			if segment.kind == 'CruiseConstantSpeed':
				if self.vehicle.configuration == 'Multirotor':
					self.default_input_values['LiftRotor|Cruise|RPM'] = [self.vehicle.lift_rotor.RPM['cruise'], 'rpm']
				elif self.vehicle.configuration == 'LiftPlusCruise':
					self.default_input_values['Propeller|Cruise|RPM'] = [self.vehicle.propeller.RPM['cruise'], 'rpm']
			if segment.kind == 'ClimbConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					self.default_input_values['LiftRotor|Climb|RPM'] = [self.vehicle.lift_rotor.RPM['climb'], 'rpm']
				elif self.vehicle.configuration == 'LiftPlusCruise':
					self.default_input_values['Propeller|Climb|RPM'] = [self.vehicle.propeller.RPM['climb'], 'rpm']
			if segment.kind == 'DescentConstantVyConstantVx':
				if self.vehicle.configuration == 'Multirotor':
					self.default_input_values['LiftRotor|Descent|RPM'] = [self.vehicle.lift_rotor.RPM['descent'], 'rpm']
				elif self.vehicle.configuration == 'LiftPlusCruise':
					self.default_input_values['Propeller|Descent|RPM'] = [self.vehicle.propeller.RPM['descent'], 'rpm']

		# Design default input values
		if self.vehicle.configuration == 'Multirotor':
			self.default_input_values['LiftRotor|radius'] = [self.vehicle.lift_rotor.radius, 'm']
			self.default_input_values['LiftRotor|hub_radius'] = [self.vehicle.lift_rotor.hub_radius, 'm']
			self.default_input_values['LiftRotor|global_twist'] = [self.vehicle.lift_rotor.global_twist, 'deg']
			self.default_input_values['LiftRotor|mean_c_to_R'] = [self.vehicle.lift_rotor.mean_c_to_R, None]
		elif self.vehicle.configuration == 'LiftPlusCruise':
			self.default_input_values['LiftRotor|radius'] = [self.vehicle.lift_rotor.radius, 'm']
			self.default_input_values['LiftRotor|hub_radius'] = [self.vehicle.lift_rotor.hub_radius, 'm']
			self.default_input_values['LiftRotor|global_twist'] = [self.vehicle.lift_rotor.global_twist, 'deg']
			self.default_input_values['LiftRotor|mean_c_to_R'] = [self.vehicle.lift_rotor.mean_c_to_R, None]
			self.default_input_values['Propeller|radius'] = [self.vehicle.propeller.radius, 'm']
			self.default_input_values['Propeller|mean_c_to_R'] = [self.vehicle.propeller.mean_c_to_R, None]
			self.default_input_values['Wing|area'] = [self.vehicle.wing.area, 'm**2']
			self.default_input_values['Wing|aspect_ratio'] = [self.vehicle.wing.aspect_ratio, None]

		# Variables needed for BEMT
		if self.fidelity['hover_climb'] == 2:
			n_sections = self.vehicle.lift_rotor.n_section
			r_to_R_list = self.vehicle.lift_rotor.r_to_R_list
			c_to_R_list = self.vehicle.lift_rotor.c_to_R_list
			w_to_R_list = self.vehicle.lift_rotor.w_to_R_list
			if self.vehicle.lift_rotor.pitch_linear_grad is not None:
				self.default_input_values['LiftRotor|pitch_linear_grad'] = [self.vehicle.lift_rotor.pitch_linear_grad, 'deg']
			else:
				pitch_list = np.array(self.vehicle.lift_rotor.pitch_list)
				for i in range(n_sections):
					self.default_input_values[f'LiftRotor|Section{i+1}|pitch'] = [pitch_list[i], 'deg']
			for i in range(n_sections):
				self.default_input_values[f'LiftRotor|Section{i+1}|r_to_R'] = [r_to_R_list[i], None]
				self.default_input_values[f'LiftRotor|Section{i+1}|c_to_R'] = [c_to_R_list[i], None]
				self.default_input_values[f'LiftRotor|Section{i+1}|w_to_R'] = [w_to_R_list[i], None]

		# Popping variables that have been used as design variables
		for var_name in self.design_variables:
			try:
				self.default_input_values.pop(var_name)
			except KeyError:
				pass

		# print(list(self.design_variables.keys()))
		# print(list(self.default_input_values.keys()))

	def evaluate_one_sample(self, x:dict):
		"""
		Given the design variable dictionary, evaluate the objective and constraints
		"""
		
		if self.vehicle.configuration == 'Multirotor':
			vehicle = StandardMultirotorEVTOL(x)

		elif self.vehicle.configuration == 'LiftPlusCruise':
			vehicle = StandardLiftPlusCruiseEVTOL(x)

		# Analysis
		analysis = WeightAnalysis(vehicle=vehicle,
								  mission=self.mission,
								  sizing_mode=True)
		
		results = analysis.evaluate()

		f = results.get_val('Weight|takeoff', 'kg')

		return f



















