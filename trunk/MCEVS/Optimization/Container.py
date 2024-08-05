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
	def __init__(self, vehicle=object, mission=object, constants=object, algorithm='str'):
		super(DesignProblem, self).__init__()
		self.kind = None # ['SingleObjectiveProblem', 'MultiObjectiveProblem']

		# MCEVS core objects
		self.vehicle 	= vehicle
		self.mission 	= mission
		self.constants 	= constants

		# Design problem
		self.objectives				= None
		self.design_variables		= {}
		self.constraints 			= {}

		# The driver
		self.algorithm		= algorithm

		# Initial, final, and optimal designs
		self.initial_design			= None
		self.final_design 			= None
		self.optimal_design			= None

	def run_optimization(self):
		"""
		Run the optimization and actually solve the problem
		"""
		if self.algorithm == 'gradient-based':
			# Run !!!
			result = run_gradient_based_optimization(self)

		elif self.algorithm == 'gradient-free':
			# Run !!!
			result = run_gradient_free_optimization(self)

		return result

	def add_objective(self, name):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					- Mission|cruise_speed
					- Wing|area
						  |aspect_ratio
					- LiftRotor|radius
							   |advance_ratio
					- Propeller|radius
							   |advance_ratio
		"""
		self.objectives = name

	def add_design_var(self, name:str, lower:float, upper:float, units=None):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					- Mission|cruise_speed
					- Wing|area
						  |aspect_ratio
					- LiftRotor|radius
							   |advance_ratio
					- Propeller|radius
							   |advance_ratio
			lower: lower bound
			upper: upper bound
			units: units
		"""
		if name == 'Mission|cruise_speed':
			for segment in self.mission.segments:
				if segment.kind == 'CruiseConstantSpeed':
					init = segment.speed
					break

		elif name == 'Wing|area':
			init = self.vehicle.wing.area

		elif name == 'Wing|aspect_ratio':
			init = self.vehicle.wing.aspect_ratio

		elif name == 'LiftRotor|radius':
			init = self.vehicle.lift_rotor.radius

		elif name == 'LiftRotor|advance_ratio':
			init = self.vehicle.lift_rotor.advance_ratio

		elif name == 'Propeller|radius':
			init = self.vehicle.propeller.radius

		elif name == 'Propeller|advance_ratio':
			init = self.vehicle.propeller.advance_ratio

		# Append design variable info
		self.design_variables[name] = [lower, upper, init, units]

	def add_constraint(self, name, lower=float, upper=float, units=None):
		"""
		Inputs:
			name: name of the variables in OpenMDAO format
				available:
					- Mission|cruise_speed
					- Wing|area
						  |aspect_ratio
					- LiftRotor|radius
							   |advance_ratio
					- Propeller|radius
							   |advance_ratio
			lower: lower bound
			upper: upper bound
			units: units
		"""
		self.constraints[name] = [lower, upper, units]		

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
								  constants=self.constants,
								  sizing_mode=True)
		
		results = analysis.evaluate()

		f = results.get_val('Weight|takeoff', 'kg')

		return f



















