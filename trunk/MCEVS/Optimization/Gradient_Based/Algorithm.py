import openmdao.api as om

from MCEVS.Analyses.Weight.Analysis import MTOWEstimation
from MCEVS.Utils.Performance import record_performance_by_segments

def run_gradient_based_optimization(DesignProblem:object):

	prob = om.Problem()

	indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

	# Initial values
	for var_name in DesignProblem.design_variables:
		indeps.add_output(var_name,
						  DesignProblem.design_variables[var_name][2],
						  units=DesignProblem.design_variables[var_name][3])
	
	# Objective function
	prob.model.add_objective(DesignProblem.objectives)

	# Design variables and their boundaries
	for var_name in DesignProblem.design_variables:
		prob.model.add_design_var(var_name,
								  lower=DesignProblem.design_variables[var_name][0],
								  upper=DesignProblem.design_variables[var_name][1],
								  units=DesignProblem.design_variables[var_name][3])

	# Constraints
	for cnstr_name in DesignProblem.constraints:
		prob.model.add_constraint(cnstr_name,
								  lower=DesignProblem.constraints[cnstr_name][0],
								  upper=DesignProblem.constraints[cnstr_name][1],
								  units=DesignProblem.constraints[cnstr_name][2])

	# Analysis
	if DesignProblem.objectives == 'Weight|takeoff':
		prob.model.add_subsystem('weight_estimation',
								  MTOWEstimation(mission=DesignProblem.mission,
								  				 vehicle=DesignProblem.vehicle,
								  				 constants=DesignProblem.constants,
								  				 sizing_mode=True),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

	# Optimizer settings
	prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, disp=True)
	prob.driver.recording_options['includes'] = ['*']
	prob.driver.recording_options['record_objectives'] = True
	prob.driver.recording_options['record_constraints'] = True
	prob.driver.recording_options['record_desvars'] = True
	prob.driver.recording_options['record_inputs'] = True
	prob.driver.recording_options['record_outputs'] = True
	prob.driver.recording_options['record_residuals'] = True

	if DesignProblem.vehicle.configuration == 'Multirotor':
		prob.driver.add_recorder(om.SqliteRecorder("multirotor_opt_cases.sql"))
	elif DesignProblem.vehicle.configuration == 'LiftPlusCruise':
		prob.driver.add_recorder(om.SqliteRecorder("liftcruise_opt_cases.sql"))

	# Run optimization
	prob.setup(check=False)
	prob.run_driver()

	# Record the optimal design performance
	record_performance_by_segments(prob, DesignProblem.vehicle.configuration, DesignProblem.mission)

	return prob