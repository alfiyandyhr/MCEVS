import openmdao.api as om

from MCEVS.Analyses.Weight.Analysis import MTOWEstimation, GTOWEstimation, OffDesignMTOWEstimation
from MCEVS.Analyses.Weight.Analysis import MultiPointMTOWEstimation, MultiPointMTOWEstimationWithFixedEmptyWeight
from MCEVS.Analyses.Geometry.Clearance import LiftRotorClearanceConstraintTypeOne, LiftRotorClearanceConstraintTypeTwo, LiftRotorClearanceConstraintTypeThree
from MCEVS.Analyses.Geometry.Rotor import MeanChord
# from MCEVS.Utils.Performance import record_performance_by_segments


def run_gradient_based_optimization(DesignProblem: object):

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
        if DesignProblem.vehicle.lift_rotor.clearance['type'] == 1:
            if DesignProblem.kind in ['SingleObjectiveProblem', 'MultiObjectiveProblem', 'GTOWSingleObjectiveProblem']:
                prob.model.add_subsystem('lift_rotor_clearance',
                                         LiftRotorClearanceConstraintTypeOne(N_rotor=DesignProblem.vehicle.lift_rotor.n_rotor,
                                                                             max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
                                                                             percent_max_span=DesignProblem.vehicle.lift_rotor.clearance['percent_max_span']),
                                         promotes_inputs=['LiftRotor|radius', 'Wing|area', 'Wing|aspect_ratio'],
                                         promotes_outputs=[('clearance_constraint', 'LiftRotor|clearance_constraint')])
            elif DesignProblem.kind in ['MultiPointSingleObjectiveProblem', 'MultiPointSingleObjectiveProblemWithFixedEmptyWeight']:
                for n in range(1, DesignProblem.multipoint_options['n_points'] + 1):
                    prob.model.add_subsystem(f'lift_rotor_clearance_{n}',
                                             LiftRotorClearanceConstraintTypeOne(N_rotor=DesignProblem.vehicle.lift_rotor.n_rotor,
                                                                                 max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
                                                                                 percent_max_span=DesignProblem.vehicle.lift_rotor.clearance['percent_max_span']),
                                             promotes_inputs=['LiftRotor|radius', 'Wing|area', 'Wing|aspect_ratio'],
                                             promotes_outputs=[('clearance_constraint', f'Point_{n}|LiftRotor|clearance_constraint')])
            elif DesignProblem.kind in ['OffDesignSingleObjectiveProblem']:
                points = ['OnDesign', 'OffDesign']
                for point in points:
                    prob.model.add_subsystem(f'lift_rotor_clearance_{point}',
                                             LiftRotorClearanceConstraintTypeOne(N_rotor=DesignProblem.vehicle.lift_rotor.n_rotor,
                                                                                 max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
                                                                                 percent_max_span=DesignProblem.vehicle.lift_rotor.clearance['percent_max_span']),
                                             promotes_inputs=['LiftRotor|radius', 'Wing|area', 'Wing|aspect_ratio'],
                                             promotes_outputs=[('clearance_constraint', f'{point}|LiftRotor|clearance_constraint')])

        elif DesignProblem.vehicle.lift_rotor.clearance['type'] == 2:
            if DesignProblem.kind in ['SingleObjectiveProblem']:
                prob.model.add_subsystem('lift_rotor_clearance',
                                         LiftRotorClearanceConstraintTypeTwo(N_rotor=DesignProblem.vehicle.lift_rotor.n_rotor,
                                                                             max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
                                                                             s_outer=DesignProblem.vehicle.lift_rotor.clearance['s_outer'],
                                                                             s_inner=DesignProblem.vehicle.lift_rotor.clearance['s_inner'],
                                                                             clearance_c=DesignProblem.vehicle.lift_rotor.clearance['clearance_c']),
                                         promotes_inputs=['LiftRotor|radius', 'Wing|area', 'Wing|aspect_ratio'],
                                         promotes_outputs=[('clearance_inner_fuselage', 'LiftRotor|clearance_inner_fuselage'), ('clearance_inner_outer', 'LiftRotor|clearance_inner_outer')])

        elif DesignProblem.vehicle.lift_rotor.clearance['type'] == 3:
            if DesignProblem.kind in ['SingleObjectiveProblem']:
                prob.model.add_subsystem('lift_rotor_clearance',
                                         LiftRotorClearanceConstraintTypeThree(max_d_fuse=DesignProblem.vehicle.fuselage.max_diameter,
                                                                               clearance_c=DesignProblem.vehicle.lift_rotor.clearance['clearance_c']),
                                         promotes_inputs=['LiftRotor|radius', 'LiftRotor|s_inner', 'LiftRotor|s_outer', 'Wing|area', 'Wing|aspect_ratio'],
                                         promotes_outputs=[('g_inner_fuse', 'LiftRotor|clearance_inner_fuselage'),
                                                           ('g_tip_outer', 'LiftRotor|clearance_tip_outer'),
                                                           ('g_inner_outer', 'LiftRotor|clearance_inner_outer'),
                                                           ('g_order', 'LiftRotor|order_constraint')])

        else:
            raise ValueError('LiftRotorClearance type should be in [1, 2, 3] !!!')

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

    if DesignProblem.kind == 'SingleObjectiveProblem':
        prob.model.add_subsystem('weight_estimation',
                                 MTOWEstimation(mission=DesignProblem.mission,
                                                vehicle=DesignProblem.vehicle,
                                                fidelity=DesignProblem.fidelity,
                                                sizing_mode=False,
                                                rhs_checking=True),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

    elif DesignProblem.kind == 'MultiPointSingleObjectiveProblem':

        prob.model.add_subsystem('multipoint_weight_estimation',
                                 MultiPointMTOWEstimation(mission=DesignProblem.mission,
                                                          vehicle=DesignProblem.vehicle,
                                                          fidelity=DesignProblem.fidelity,
                                                          multipoint_options=DesignProblem.multipoint_options),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

    elif DesignProblem.kind == 'MultiPointSingleObjectiveProblemWithFixedEmptyWeight':

        prob.model.add_subsystem('multipoint_weight_estimation',
                                 MultiPointMTOWEstimationWithFixedEmptyWeight(mission=DesignProblem.mission,
                                                                              vehicle=DesignProblem.vehicle,
                                                                              fidelity=DesignProblem.fidelity,
                                                                              multipoint_options=DesignProblem.multipoint_options),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

    elif DesignProblem.kind == 'OffDesignSingleObjectiveProblem':

        prob.model.add_subsystem('off_design_weight_estimation',
                                 OffDesignMTOWEstimation(mission=DesignProblem.mission,
                                                         vehicle=DesignProblem.vehicle,
                                                         fidelity=DesignProblem.fidelity,
                                                         offdesign_options=DesignProblem.offdesign_options),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

    elif DesignProblem.kind == 'GTOWSingleObjectiveProblem':

        indeps.add_output('Weight|propulsion', DesignProblem.vehicle.weight.propulsion, units='kg')
        indeps.add_output('Weight|structure', DesignProblem.vehicle.weight.structure, units='kg')
        indeps.add_output('Weight|equipment', DesignProblem.vehicle.weight.equipment, units='kg')
        prob.model.add_subsystem('weight_estimation',
                                 GTOWEstimation(mission=DesignProblem.mission,
                                                vehicle=DesignProblem.vehicle,
                                                fidelity=DesignProblem.fidelity,
                                                sizing_mode=False,
                                                rhs_checking=True),
                                 promotes_inputs=['*'],
                                 promotes_outputs=['*'])

    else:
        raise NotImplementedError('Please check your "DesignProblem.objectives"')

    # Optimizer settings
    # debug_print = ['desvars', 'nl_cons', 'objs']
    # debug_print = ['desvars', 'nl_cons', 'objs', 'totals']
    # debug_print = ['objs', 'desvars', 'nl_cons']
    debug_print = []
    prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-3, singular_jac_tol=1e-16, debug_print=debug_print, disp=True)
    # prob.driver = om.ScipyOptimizeDriver(optimizer='shgo', tol=1e-3, singular_jac_tol=1e-16, debug_print=debug_print, disp=True)
    # prob.driver = om.pyOptSparseDriver(optimizer='', print_opt_prob=True)
    # prob.driver = om.DifferentialEvolutionDriver(max_gen=5)
    # prob.driver.recording_options['includes'] = ['*']
    # prob.driver.recording_options['record_objectives'] = True
    # prob.driver.recording_options['record_constraints'] = True
    # prob.driver.recording_options['record_desvars'] = True
    # prob.driver.recording_options['record_inputs'] = True
    # prob.driver.recording_options['record_outputs'] = True
    # prob.driver.recording_options['record_residuals'] = True

    # if DesignProblem.vehicle.configuration == 'Multirotor':
    # 	prob.driver.add_recorder(om.SqliteRecorder("multirotor_opt_cases.sql"))
    # elif DesignProblem.vehicle.configuration == 'LiftPlusCruise':
    # 	prob.driver.add_recorder(om.SqliteRecorder("liftcruise_opt_cases.sql"))

    # Run optimization
    prob.setup(check=False)
    res_info = prob.run_driver()

    # prob.list_problem_vars(driver_scaling=False)

    # Record the optimal design performance
    # record_performance_by_segments(prob, DesignProblem.vehicle.configuration, DesignProblem.mission)

    return prob, res_info
