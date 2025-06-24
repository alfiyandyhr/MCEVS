from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunMultiPointSingleObjectiveOptimization
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
import numpy as np
import pandas as pd
import warnings
import sys
import matplotlib.pyplot as plt

do_optimizaton = False
do_test = False

do_plot_objective = True
do_plot_constraints = True

battery_energy_density_list = np.arange(250, 725, 25)
selected_battery_energy_density_list = [250.0, 400.0, 550.0]

weight_coeffs_options = {'uniform': [0.25, 0.25, 0.25, 0.25],
                         'linear': [0.10, 0.20, 0.30, 0.40]}

# Fixed mission requirement
mission_range = 60 * 1609.344  # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s

solution_fidelity = {'aero': 1, 'hover_climb': 0}

# --- Multipoint optimization --- #
if do_optimizaton:

    for i, coeff_type in enumerate(weight_coeffs_options):

        print(f'Multipoint optimization with coeff_type= {coeff_type}')
        sys.stdout.flush()  # To flush the above print output

        # Standard vehicle
        design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
        operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
        tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
        vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

        # Standard mission
        mission = StandardMissionProfile(mission_range, cruise_speed)

        # Standard optimization
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            results = {'coeff_type': f'{coeff_type}'}
            results.update(RunMultiPointSingleObjectiveOptimization(type='battery_energy_density',
                                                                    value_list=[250.0, 400.0, 550.0, 700.0],
                                                                    objective='weighted_sum_of_takeoff_weight',
                                                                    weight_coeffs=weight_coeffs_options[f'{coeff_type}'],
                                                                    with_fixed_empty_weight=True,
                                                                    vehicle=vehicle,
                                                                    mission=mission,
                                                                    fidelity=solution_fidelity,
                                                                    mtow_guess_list=[1000.0, 1000.0, 1000.0, 1000.0],
                                                                    speed_as_design_var=False,
                                                                    print=True))

            results_df = pd.DataFrame(results, index=[i])
            results_df.to_csv('multipoint_opt_results.csv', mode='a', header=True if i == 0 else False)

# --- Optimization at the assumed battery technology --- #
if do_test:

    multipoint_opt_results = pd.read_csv('multipoint_opt_results.csv')
    opt_results = pd.read_csv('../../../../4_suboptimal_performance_evaluation/with_fixed_empty_weight/selection_by_weight/multirotor/optimization_results.csv')

    for i, coeff_type in enumerate(weight_coeffs_options):

        for j, battery_energy_density in enumerate(battery_energy_density_list):

            print(f'Multipoint design with with coeff_type= {coeff_type}; tested at {battery_energy_density} Wh/kg')
            sys.stdout.flush()  # To flush the above print output

            r_lift_rotor = multipoint_opt_results[multipoint_opt_results['coeff_type'] == coeff_type]['LiftRotor|radius'].to_numpy()[0]
            RPM_lift_rotor_cruise = multipoint_opt_results[multipoint_opt_results['coeff_type'] == coeff_type]['LiftRotor|Cruise|RPM'].to_numpy()[0]
            cruise_speed = multipoint_opt_results[multipoint_opt_results['coeff_type'] == coeff_type]['cruise_speed'].to_numpy()[0]

            # Parasite drag of reference vehicle
            f_non_hub = opt_results[opt_results['battery_energy_density'] == 250]['Aero|Cruise|f_total_non_hub'].to_numpy()[0]

            # Standard vehicle
            design_var = {'r_lift_rotor': r_lift_rotor}
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': RPM_lift_rotor_cruise}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Original battery density
            vehicle.battery.density = float(selected_battery_energy_density_list[0])

            # Fixed assumed parasite drag from the ref vehicle
            vehicle.f_total_non_hub = {'climb': None, 'cruise': f_non_hub, 'descent': None}

            # Standard mission
            mission = StandardMissionProfile(mission_range, cruise_speed * 1000 / 3600)

            # MTOW Analysis
            analysis = WeightAnalysis(vehicle, mission, solution_fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
            res = analysis.evaluate(record=False, weight_guess=2500.0, print=False)

            # Now swap the battery
            vehicle.battery.density = float(battery_energy_density)

            # GTOW Analysis
            gtow_analysis = WeightAnalysis(vehicle, mission, solution_fidelity, weight_type='gross', sizing_mode=True, solved_by='optimization')
            res = gtow_analysis.evaluate(record=False, weight_guess=2500.0, print=False)

            # Bookkeeping results
            results = {'coeff_type': coeff_type, 'battery_energy_density_test': battery_energy_density}

            # Mission requirements
            results['mission_range'] = mission.segments[2].distance / 1000.0 	# km
            results['cruise_speed'] = res.get_val('Mission|segment_3|speed', 'km/h')[0]
            results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]

            # Design objective, variables, and constraints
            results['Weight|max_takeoff'] = vehicle.weight.max_takeoff
            results['Weight|gross_takeoff'] = vehicle.weight.gross_takeoff
            results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
            results['LiftRotor|Cruise|RPM'] = res.get_val('LiftRotor|Cruise|RPM', 'rpm')[0]
            results['LiftRotor|Cruise|CT/sigma'] = res.get_val('LiftRotor|Cruise|thrust_coefficient')[0] / vehicle.lift_rotor.solidity
            results['LiftRotor|Cruise|mu'] = res.get_val('LiftRotor|Cruise|mu')[0]
            results['LiftRotor|HoverClimb|T_to_P'] = res.get_val('LiftRotor|HoverClimb|T_to_P')[0]
            results['LiftRotor|Cruise|T_to_P'] = res.get_val('LiftRotor|Cruise|T_to_P')[0]
            results['LiftRotor|HoverDescent|T_to_P'] = res.get_val('LiftRotor|HoverDescent|T_to_P')[0]
            results['Weight|residual'] = res.get_val('Weight|residual', 'kg')[0]

            # Other importants results
            results['Weight|payload'] = res.get_val('Weight|payload', 'kg')[0]
            results['Weight|battery'] = res.get_val('Weight|battery', 'kg')[0]
            results['Weight|propulsion'] = res.get_val('Weight|propulsion', 'kg')[0]
            results['Weight|structure'] = res.get_val('Weight|structure', 'kg')[0]
            results['Weight|equipment'] = res.get_val('Weight|equipment', 'kg')[0]
            results['Power|segment_1'] = res.get_val('Power|segment_1', 'kW')[0]
            results['Power|segment_3'] = res.get_val('Power|segment_3', 'kW')[0]
            results['Power|segment_5'] = res.get_val('Power|segment_5', 'kW')[0]
            results['DiskLoading|LiftRotor|segment_1'] = res.get_val('DiskLoading|LiftRotor|segment_1', 'N/m**2')[0]
            results['DiskLoading|LiftRotor|segment_3'] = res.get_val('DiskLoading|LiftRotor|segment_3', 'N/m**2')[0]
            results['DiskLoading|LiftRotor|segment_5'] = res.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2')[0]
            results['Energy|entire_mission'] = res.get_val('Energy|entire_mission', 'kW*h')[0]

            # Aerodynamics at cruise
            results['Aero|Cruise|f_total_non_hub'] = vehicle.f_total_non_hub['cruise']
            results['Aero|Cruise|f_total'] = res.get_val('Aero|Cruise|f_total', 'm**2')[0]
            results['Aero|Cruise|total_drag'] = res.get_val('Aero|Cruise|total_drag', 'N')[0]

            # Saving to csv file
            results_df = pd.DataFrame(results, index=[i * 2 + j])
            results_df.to_csv('multipoint_test_results.csv', mode='a', header=True if i * 2 + j == 0 else False)

if do_plot_objective:
    opt_results = pd.read_csv('../../../../4_suboptimal_performance_evaluation/with_fixed_empty_weight/selection_by_weight/multirotor/optimization_results.csv')
    test_results = pd.read_csv('../../../../4_suboptimal_performance_evaluation/with_fixed_empty_weight/selection_by_weight/multirotor/test_results.csv')
    multipoint_test_results = pd.read_csv('multipoint_test_results.csv')

    ylabel = 'Weight|gross_takeoff'

    # Processing constraints
    opt_results.loc[opt_results['Weight|residual'] > 0.1, ylabel] = np.nan
    opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, ylabel] = np.nan
    opt_results.loc[opt_results['LiftRotor|Cruise|T_to_P'] > 12.01, ylabel] = np.nan
    opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, ylabel] = np.nan
    opt_results.loc[opt_results['LiftRotor|Cruise|mu'] > 1, ylabel] = np.nan
    opt_results.loc[opt_results['LiftRotor|Cruise|CT/sigma'] > 0.141, ylabel] = np.nan

    test_results['Weight|gross_takeoff'] = np.where((test_results['Weight|max_takeoff'] < test_results['Weight|gross_takeoff']) & (test_results['battery_energy_density_opt'] != test_results['battery_energy_density_test']), np.nan, test_results['Weight|gross_takeoff'])
    test_results.loc[test_results['Weight|residual'] > 0.1, ylabel] = np.nan
    test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, ylabel] = np.nan
    test_results.loc[test_results['LiftRotor|Cruise|T_to_P'] > 12.01, ylabel] = np.nan
    test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, ylabel] = np.nan
    test_results.loc[test_results['LiftRotor|Cruise|mu'] > 1, ylabel] = np.nan
    test_results.loc[test_results['LiftRotor|Cruise|CT/sigma'] > 0.141, ylabel] = np.nan

    multipoint_test_results['Weight|gross_takeoff'] = np.where((multipoint_test_results['Weight|max_takeoff'] < multipoint_test_results['Weight|gross_takeoff']) & (multipoint_test_results['battery_energy_density_test'] != 250), np.nan, multipoint_test_results['Weight|gross_takeoff'])
    multipoint_test_results.loc[multipoint_test_results['Weight|residual'] > 0.1, ylabel] = np.nan
    multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, ylabel] = np.nan
    multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|T_to_P'] > 12.01, ylabel] = np.nan
    multipoint_test_results.loc[multipoint_test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, ylabel] = np.nan
    multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|mu'] > 1, ylabel] = np.nan
    multipoint_test_results.loc[multipoint_test_results['LiftRotor|Cruise|CT/sigma'] > 0.141, ylabel] = np.nan

    for i, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
        test_results_i = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][ylabel]
        plt.plot(battery_energy_density_list, test_results_i, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[i]} Wh/kg')
    for j, coeff_type in enumerate(weight_coeffs_options):
        if coeff_type == 'uniform':
            multipoint_test_results_j = multipoint_test_results[multipoint_test_results['coeff_type'] == coeff_type][ylabel]
            plt.plot(battery_energy_density_list, multipoint_test_results_j, 'o-', ms=4.0, label=f'Multipoint coeff={coeff_type}')
        if coeff_type == 'linear':
            multipoint_test_results_j = multipoint_test_results[multipoint_test_results['coeff_type'] == coeff_type][ylabel]
            plt.plot(battery_energy_density_list, multipoint_test_results_j, 'x-', label=f'Multipoint coeff={coeff_type}')
    plt.plot(opt_results['battery_energy_density'], opt_results['Weight|takeoff'], 'k--', ms=4.0, label='Optimized at test battery GED')
    plt.title('Suboptimal performance evaluation\nfor Multirotor from weight-based optimization')
    plt.xlabel(r'Test battery GED $[Wh/kg]$')
    plt.ylabel(r'Takeoff weight $[kg]$')
    plt.legend()
    plt.show()

if do_plot_constraints:
    test_results = pd.read_csv('../../../../4_suboptimal_performance_evaluation/with_fixed_empty_weight/selection_by_weight/multirotor/test_results.csv')
    multipoint_test_results = pd.read_csv('multipoint_test_results.csv')

    xmin = battery_energy_density_list[0]
    xmax = battery_energy_density_list[-1]

    ylabels = ['LiftRotor|Cruise|CT/sigma', 'LiftRotor|Cruise|mu', 'Weight|residual',
               'LiftRotor|HoverClimb|T_to_P', 'LiftRotor|Cruise|T_to_P', 'LiftRotor|HoverDescent|T_to_P']
    ylabel_names = [r'Blade loading $CT/\sigma$', r'Advance ratio $\mu$', r'Squared residual $[kg^2]$', r'$T/P$ HoverClimb $[g/W]$', r'$T/P$ Cruise $[g/W]$', r'$T/P$ HoverDescent $[g/W]$']

    fig, axes = plt.subplots(2, 3, sharex=True, figsize=(12, 7))
    axes = axes.flatten()

    for j, ylabel in enumerate(ylabels):
        for i, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
            test_results_i = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][ylabel]
            axes[j].plot(battery_energy_density_list, test_results_i, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if j == 0 else None)
            if ylabel == 'LiftRotor|Cruise|CT/sigma':
                axes[j].hlines(0.14, xmin=xmin, xmax=xmax, linestyles='--', color='k', label='Constraint upper limit' if i == 2 else None)
            if ylabel == 'LiftRotor|Cruise|mu':
                axes[j].hlines(1.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
            if ylabel == 'LiftRotor|HoverClimb|T_to_P':
                axes[j].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
            if ylabel == 'LiftRotor|Cruise|T_to_P':
                axes[j].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
            if ylabel == 'LiftRotor|HoverDescent|T_to_P':
                axes[j].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
            if j in [3, 4, 5]:
                axes[j].set_xlabel('Test battery GED [Wh/kg]')
            axes[j].set_ylabel(ylabel_names[j])
        for k, coeff_type in enumerate(weight_coeffs_options):
            if coeff_type == 'uniform':
                multipoint_test_results_k = multipoint_test_results[multipoint_test_results['coeff_type'] == coeff_type][ylabel]
                axes[j].plot(battery_energy_density_list, multipoint_test_results_k, 'o-', ms=3.0, label=f'Multipoint coeff={coeff_type}' if j == 0 else None)
            if coeff_type == 'linear':
                multipoint_test_results_k = multipoint_test_results[multipoint_test_results['coeff_type'] == coeff_type][ylabel]
                axes[j].plot(battery_energy_density_list, multipoint_test_results_k, 'x-', label=f'Multipoint coeff={coeff_type}' if j == 0 else None)
    axes[2].ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
    fig.suptitle('Constraint evaluation with varying battery GED for Multirotor from weight-based optimization')
    fig.legend(ncols=3, bbox_to_anchor=(0.50, 0.95), loc='upper center')
    # plt.subplots_adjust(left=0.07, bottom=0.1, right=0.93, top=0.83, hspace=0.08)
    plt.tight_layout(rect=(0, 0, 1, 0.93))
    plt.show()
