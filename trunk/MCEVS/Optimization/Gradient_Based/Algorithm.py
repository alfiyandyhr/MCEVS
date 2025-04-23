import openmdao.api as om

from MCEVS.Analyses.Weight.Analysis import MTOWEstimation
from MCEVS.Analyses.Geometry.Clearance import LiftRotorClearanceConstraint
from MCEVS.Analyses.Geometry.Rotor import MeanChord
from MCEVS.Utils.Performance import record_performance_by_segments

def run_gradient_based_optimization(DesignProblem:object):

	prob = om.Problem(reports=False)

	indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

	# Default input values
	for input_name in DesignProblem.default_input_values:
		indeps.add_output(input_name,
						  DesignProblem.default_input_values[input_name][0],
						  units=DesignProblem.default_input_values[input_name][1])

	# Initial values
	for var_name in DesignProblem.design_variables:
		indeps.add_output(var_name,
						  DesignProblem.design_variables[var_name][2],
						  units=DesignProblem.design_variables[var_name][3])
	
	# Objective function
	for obj_name in DesignProblem.objectives:
		prob.model.add_objective(DesignProblem.objectives[obj_name][0],
								 ref=DesignProblem.objectives[obj_name][1],
								 units=DesignProblem.objectives[obj_name][2])

	# Design variables and their boundaries
	for var_name in DesignProblem.design_variables:
		prob.model.add_design_var(var_name,
								  lower=DesignProblem.design_variables[var_name][0],
								  upper=DesignProblem.design_variables[var_name][1],
								  ref=DesignProblem.design_variables[var_name][2],
								  units=DesignProblem.design_variables[var_name][3])

	# Constraints
	for cnstr_name in DesignProblem.constraints:
		prob.model.add_constraint(cnstr_name,
								  lower=DesignProblem.constraints[cnstr_name][0],
								  upper=DesignProblem.constraints[cnstr_name][1],
								  ref=DesignProblem.constraints[cnstr_name][2],
								  units=DesignProblem.constraints[cnstr_name][3])

	# Geometric analysis
	if DesignProblem.vehicle.configuration == 'Multirotor':
		# Convert mean_c_to_R into mean_chord
		prob.model.add_subsystem('chord_calc_lift_rotor',
								  MeanChord(),
								  promotes_inputs=[('mean_c_to_R', 'LiftRotor|mean_c_to_R'), ('R', 'LiftRotor|radius')],
								  promotes_outputs=[('mean_chord', 'LiftRotor|chord')])

	elif DesignProblem.vehicle.configuration == 'LiftPlusCruise':
		# Calculate spanwise clearance constraint for lift rotor of LPC config
		prob.model.add_subsystem('lift_rotor_clearance',
								  LiftRotorClearanceConstraint(N_rotor=DesignProblem.vehicle.lift_rotor.n_rotor,
								  							   max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
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

	# Analysis

	if list(DesignProblem.objectives.keys())[0] in ['Weight|takeoff', 'Weight|residual', 'Weight|battery', 'Energy|entire_mission', 'Mission|total_time']:
		prob.model.add_subsystem('weight_estimation',
								  MTOWEstimation(mission=DesignProblem.mission,
								  				 vehicle=DesignProblem.vehicle,
								  				 fidelity=DesignProblem.fidelity,
								  				 sizing_mode=False),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])
	else:
		raise NotImplementedError('Please check your "DesignProblem.objectives"')

	# Optimizer settings
	# debug_print = ['desvars', 'nl_cons', 'objs']
	# debug_print = ['desvars', 'nl_cons', 'objs', 'totals']
	debug_print = []
	prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, singular_jac_tol=1e-16, debug_print=debug_print, disp=True)
	# prob.driver = om.pyOptSparseDriver(optimizer='', print_opt_prob=True)
	# prob.driver = om.DifferentialEvolutionDriver(max_gen=5)
	prob.driver.recording_options['includes'] = ['*']
	prob.driver.recording_options['record_objectives'] = True
	prob.driver.recording_options['record_constraints'] = True
	prob.driver.recording_options['record_desvars'] = True
	prob.driver.recording_options['record_inputs'] = True
	prob.driver.recording_options['record_outputs'] = True
	prob.driver.recording_options['record_residuals'] = True

	# if DesignProblem.vehicle.configuration == 'Multirotor':
	# 	prob.driver.add_recorder(om.SqliteRecorder("multirotor_opt_cases.sql"))
	# elif DesignProblem.vehicle.configuration == 'LiftPlusCruise':
	# 	prob.driver.add_recorder(om.SqliteRecorder("liftcruise_opt_cases.sql"))

	# Run optimization
	prob.setup(check=False)
	prob.run_driver()

	# Record the optimal design performance
	record_performance_by_segments(prob, DesignProblem.vehicle.configuration, DesignProblem.mission)

	return prob