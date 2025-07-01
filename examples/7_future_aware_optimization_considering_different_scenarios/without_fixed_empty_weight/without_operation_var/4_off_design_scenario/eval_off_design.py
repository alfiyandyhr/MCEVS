from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.integrate import simpson
import sys

eval_off_design_performace = False
plot_off_design_performance = True
savefig = False

# 40 year lifespan of the product
year_list = np.arange(2030, 2071, 1)

# Fixed mission requirement
mission_range = 60 * 1609.344  # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'},
            'weight_model': {'structure': 'Roskam'},
            'stability': {'AoA_trim': {'cruise': 'ManualFixedValue'}}}


def Boltzman_Sigmoid_Battery_Pack_GED(t, scenario):

    if scenario == 'conservative':
        models = [9.6, 659.1, 0.0436, 2042.5]
        derating_factor = 0.7
    elif scenario == 'nominal':
        models = [0.0, 801.8, 0.0607, 2030.8]
        derating_factor = 0.8
    elif scenario == 'aggresive':
        models = [32.5, 1009.0, 0.0786, 2030.6]
        derating_factor = 0.9

    a = models[0]
    b = models[1]
    c = models[2]
    tau = models[3]
    res = a + b / (1 + np.exp(c * (tau - t)))

    return derating_factor * res


# Battery dictionary
battery_dict = {'conservative': None, 'nominal': None, 'aggresive': None}

for i, scenario in enumerate(battery_dict):
    battery_dict[scenario] = Boltzman_Sigmoid_Battery_Pack_GED(year_list, scenario)


if eval_off_design_performace:

    utopian_data = pd.read_csv('../1_utopian_data/optimal_results_by_scenario_by_year.csv')

    iter_idx = 0
    for i, opt_scenario in enumerate(battery_dict):

        if opt_scenario == 'conservative':
            min_psi_year = 2043
        if opt_scenario == 'nominal':
            min_psi_year = 2044
        if opt_scenario == 'aggresive':
            min_psi_year = 2041

        opt_results = utopian_data[utopian_data['scenario'] == opt_scenario]
        battery_energy_density_opt = opt_results[opt_results['year'] == min_psi_year]['battery_energy_density'].to_numpy()[0]

        wing_area = opt_results[opt_results['year'] == min_psi_year]['Wing|area'].to_numpy()[0]
        wing_aspect_ratio = opt_results[opt_results['year'] == min_psi_year]['Wing|aspect_ratio'].to_numpy()[0]
        r_lift_rotor = opt_results[opt_results['year'] == min_psi_year]['LiftRotor|radius'].to_numpy()[0]
        r_propeller = opt_results[opt_results['year'] == min_psi_year]['Propeller|radius'].to_numpy()[0]
        RPM_propeller_cruise = opt_results[opt_results['year'] == min_psi_year]['Propeller|Cruise|RPM'].to_numpy()[0]
        cruise_speed = opt_results[opt_results['year'] == min_psi_year]['cruise_speed'].to_numpy()[0]

        # Parasite drag of reference vehicle
        f_non_hub_non_wing = opt_results[opt_results['year'] == min_psi_year]['Aero|Cruise|f_total_non_hub_non_wing'].to_numpy()[0]
        Cd0_wing = opt_results[opt_results['year'] == min_psi_year]['Aero|Cruise|Cd0_wing'].to_numpy()[0]

        for j, test_scenario in enumerate(battery_dict):

            battery_energy_density_test_list = battery_dict[test_scenario]

            for k, sizing_year in enumerate(year_list):

                iter_idx += 1

                battery_energy_density_test = battery_energy_density_test_list[k]

                print(f'No={iter_idx}; Opt scenario={opt_scenario}; Test scenario= {test_scenario}; Optimized in Year={min_psi_year}; Tested in Year={sizing_year}')
                sys.stdout.flush()  # To flush the above print output

                # Standard vehicle
                design_var = {'wing_area': wing_area, 'wing_aspect_ratio': wing_aspect_ratio, 'r_lift_rotor': r_lift_rotor, 'r_propeller': r_propeller}
                operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': RPM_propeller_cruise}}
                tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
                vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

                # Changed battery density
                vehicle.battery.density = float(battery_energy_density_test)

                # Fixed assumed parasite drag from the ref vehicle
                vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}
                vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}

                # Standard mission
                mission = StandardMissionProfile(mission_range, cruise_speed * 1000 / 3600)

                # MTOWEstimation at test year
                analysis = WeightAnalysis(vehicle, mission, fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
                res = analysis.evaluate(record=False, weight_guess=2500.0, print=False)

                # Bookkeeping results
                results = {'opt_scenario': opt_scenario,
                           'test_scenario': test_scenario,
                           'year_opt': min_psi_year,
                           'year_test': sizing_year,
                           'battery_energy_density_opt': battery_energy_density_opt,
                           'battery_energy_density_test': battery_energy_density_test}

                # Mission requirements
                results['mission_range'] = mission.segments[2].distance / 1000.0    # km
                results['cruise_speed'] = res.get_val('Mission|segment_3|speed', 'km/h')[0]
                results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]

                # Design objective, variables, and constraints
                results['Weight|takeoff'] = res.get_val('Weight|takeoff', 'kg')[0]
                results['Wing|area'] = res.get_val('Wing|area', 'm**2')[0]
                results['Wing|aspect_ratio'] = res.get_val('Wing|aspect_ratio', None)[0]
                results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
                results['Propeller|radius'] = res.get_val('Propeller|radius', 'm')[0]
                results['Propeller|Cruise|RPM'] = res.get_val('Propeller|Cruise|RPM', 'rpm')[0]
                results['Aero|Cruise|CL'] = res.get_val('Aero|Cruise|CL', None)[0]
                results['Propeller|Cruise|CT/sigma'] = res.get_val('Propeller|Cruise|thrust_coefficient')[0] / vehicle.propeller.solidity
                results['Propeller|Cruise|J'] = res.get_val('Propeller|Cruise|J')[0]
                results['LiftRotor|HoverClimb|T_to_P'] = res.get_val('LiftRotor|HoverClimb|T_to_P')[0]
                results['Propeller|Cruise|T_to_P'] = res.get_val('Propeller|Cruise|T_to_P')[0]
                results['LiftRotor|HoverDescent|T_to_P'] = res.get_val('LiftRotor|HoverDescent|T_to_P')[0]
                results['LiftRotor|clearance_constraint'] = res.get_val('LiftRotor|clearance_constraint')[0]
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
                results['DiskLoading|Propeller|segment_3'] = res.get_val('DiskLoading|Propeller|segment_3', 'N/m**2')[0]
                results['DiskLoading|LiftRotor|segment_5'] = res.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2')[0]
                results['Energy|entire_mission'] = res.get_val('Energy|entire_mission', 'kW*h')[0]

                # Aerodynamics at cruise
                results['Aero|Cruise|f_total_non_hub_non_wing'] = vehicle.f_total_non_hub_non_wing['cruise']
                results['Aero|Cruise|Cd0_wing'] = vehicle.wing.Cd0['cruise']
                results['Aero|Cruise|f_total'] = res.get_val('Aero|Cruise|f_total', 'm**2')[0]
                results['Aero|Cruise|total_drag'] = res.get_val('Aero|Cruise|total_drag', 'N')[0]

                # Saving to csv file
                results_df = pd.DataFrame(results, index=[iter_idx])
                results_df.to_csv('off_design_results.csv', mode='a', header=True if iter_idx == 1 else False)

if plot_off_design_performance:

    off_design_results = pd.read_csv('off_design_results.csv')

    psi_df = pd.DataFrame({'opt_scenario': pd.Series(dtype='str'), 'test_scenario': pd.Series(dtype='str'), 'psi': pd.Series(dtype='float')})

    for i, opt_scenario in enumerate(battery_dict):

        off_design_results_i = off_design_results[off_design_results['opt_scenario'] == opt_scenario]

        for j, test_scenario in enumerate(battery_dict):

            y_array = off_design_results_i[off_design_results_i['test_scenario'] == test_scenario]['Energy|entire_mission'].to_numpy()

            time_averaged_J = simpson(y_array, year_list) / (year_list[-1] - year_list[0])

            if test_scenario == 'conservative':
                time_averaged_utopian_J = 103.11640095639873
            if test_scenario == 'nominal':
                time_averaged_utopian_J = 88.53419623780462
            if test_scenario == 'aggresive':
                time_averaged_utopian_J = 83.01572130903654

            psi_ij = time_averaged_J / time_averaged_utopian_J - 1

            new_psi_df = pd.DataFrame({'opt_scenario': [opt_scenario], 'test_scenario': [test_scenario], 'psi': [psi_ij]})
            psi_df = pd.concat([psi_df, new_psi_df], ignore_index=True)

    plt.figure(figsize=(8, 5))
    sns.lineplot(data=psi_df, x="test_scenario", y="psi", hue="opt_scenario", marker='o')
    plt.xlabel("Test Scenario")
    plt.ylabel(r"Future-aware optimality metric $\Psi$")
    plt.title(r"Off-design performance of $\Psi$-minimal designs")
    plt.legend(title="Optimized Scenario")
    plt.tight_layout()
    plt.savefig('off_design_performance_future_aware.pdf', format='pdf', dpi=300) if savefig else plt.show()
