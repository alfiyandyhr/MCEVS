from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL, StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
import numpy as np
import pandas as pd
import warnings
import sys
import matplotlib.pyplot as plt

do_weight_optimizaton_multirotor = False
do_weight_optimizaton_liftcruise = False
do_energy_optimizaton_multirotor = False
do_energy_optimizaton_liftcruise = False
plot_utopian_data = True

# Fixed mission requirement
mission_range = 60 * 1609.344  # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s

solution_fidelity = {'aero': 1, 'hover_climb': 0}


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

# Populating data
year_list = np.arange(2030, 2071, 1)
for i, scenario in enumerate(battery_dict):
    battery_dict[scenario] = Boltzman_Sigmoid_Battery_Pack_GED(year_list, scenario)

if do_weight_optimizaton_multirotor:

    for i, scenario in enumerate(battery_dict):

        for j, year in enumerate(year_list):

            k = 41 * i + j

            battery_energy_density = battery_dict[scenario][j]

            # if scenario == 'conservative' and year in [2030, 2031, 2032]:

            print(f'{k}, Scenario= {scenario}; Year= {year}; Battery GED= {battery_energy_density} Wh/kg')
            sys.stdout.flush()  # To flush the above print output

            if scenario == 'aggresive':
                mtow_guess = 1000.0  # kg

            if scenario == 'nominal':
                mtow_guess = 1000.0  # kg

            if scenario == 'conservative':
                mtow_guess = 1000.0  # kg

            # Standard vehicle
            design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if k != 0:
                vehicle.f_total_non_hub = {'climb': None, 'cruise': f_non_hub, 'descent': None}  # noqa: F821

            # Standard mission
            mission = StandardMissionProfile(mission_range, cruise_speed)

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = {'scenario': scenario, 'year': year, 'battery_energy_density': battery_energy_density}
                results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=True))
                results_df = pd.DataFrame(results, index=[k])
                results_df.to_csv('optimal_results_by_scenario_by_year/weight_opt_multirotor_results.csv', mode='a', header=True if k == 0 else False)

            if k == 0:
                f_non_hub = results['Aero|Cruise|f_total_non_hub']

if do_weight_optimizaton_liftcruise:

    for i, scenario in enumerate(battery_dict):

        for j, year in enumerate(year_list):

            k = 41 * i + j

            battery_energy_density = battery_dict[scenario][j]

            print(f'{k}, Scenario= {scenario}; Year= {year}; Battery GED= {battery_energy_density} Wh/kg')
            sys.stdout.flush()  # To flush the above print output

            if scenario == 'aggresive':
                # if year in [2031,2032,2033,2035,2037,2038,2041,2045,2046,2051,2055,2056,2057,2061,2065,2068]:
                # 	mtow_guess = 1500.0 # kg
                # elif year in [2047,2048]:
                # 	mtow_guess = 1200.0
                # else:
                mtow_guess = 1000.0  # kg

            if scenario == 'nominal':
                # if year in [2037,2038,2046,2050,2051,2055,2061,2062,2065,2070]:
                # 	mtow_guess = 2000.0 # kg
                # elif year in [2031,2034,2049,2058,2059,2064]:
                # 	mtow_guess = 1500.0 # kg
                # elif year in [2032]:
                # 	mtow_guess = 1200.0 # kg
                # else:
                mtow_guess = 1000.0  # kg

            if scenario == 'conservative':
                # if year in [2063, 2068]:
                # 	mtow_guess = 1500.0 # kg
                # else:
                mtow_guess = 1000.0  # kg

            # Standard vehicle
            design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if k != 0:
                vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}  # noqa: F821
                vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}  # noqa: F821

            # Standard mission
            mission = StandardMissionProfile(mission_range, cruise_speed)

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = {'scenario': scenario, 'year': year, 'battery_energy_density': battery_energy_density}
                results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=True))
                results_df = pd.DataFrame(results, index=[k])
                results_df.to_csv('optimal_results_by_scenario_by_year/weight_opt_liftcruise_results.csv', mode='a', header=True if k == 0 else False)

            if k == 0:
                f_non_hub_non_wing = results['Aero|Cruise|f_total_non_hub_non_wing']
                Cd0_wing = results['Aero|Cruise|Cd0_wing']

if do_energy_optimizaton_multirotor:

    for i, scenario in enumerate(battery_dict):

        for j, year in enumerate(year_list):

            k = 41 * i + j

            battery_energy_density = battery_dict[scenario][j]

            # if scenario == 'conservative' and year in [2030, 2031, 2032]:

            print(f'{k}, Scenario= {scenario}; Year= {year}; Battery GED= {battery_energy_density} Wh/kg')
            sys.stdout.flush()  # To flush the above print output

            if scenario == 'aggresive':
                mtow_guess = 1000.0  # kg

            if scenario == 'nominal':
                mtow_guess = 1000.0  # kg

            if scenario == 'conservative':
                mtow_guess = 1000.0  # kg

            # Standard vehicle
            design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if k != 0:
                vehicle.f_total_non_hub = {'climb': None, 'cruise': f_non_hub, 'descent': None}

            # Standard mission
            mission = StandardMissionProfile(mission_range, cruise_speed)

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = {'scenario': scenario, 'year': year, 'battery_energy_density': battery_energy_density}
                results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=True))
                results_df = pd.DataFrame(results, index=[k])
                results_df.to_csv('optimal_results_by_scenario_by_year/energy_opt_multirotor_results.csv', mode='a', header=True if k == 0 else False)

            if k == 0:
                f_non_hub = results['Aero|Cruise|f_total_non_hub']

if do_energy_optimizaton_liftcruise:

    for i, scenario in enumerate(battery_dict):

        for j, year in enumerate(year_list):

            k = 41 * i + j

            battery_energy_density = battery_dict[scenario][j]

            print(f'{k}, Scenario= {scenario}; Year= {year}; Battery GED= {battery_energy_density} Wh/kg')
            sys.stdout.flush()  # To flush the above print output

            if scenario == 'aggresive':
                if year in [2031, 2032, 2033, 2035, 2037, 2038, 2041, 2045, 2046, 2051, 2055, 2056, 2057, 2061, 2065, 2068]:
                    mtow_guess = 1500.0  # kg
                elif year in [2047, 2048]:
                    mtow_guess = 1200.0
                else:
                    mtow_guess = 1000.0  # kg

            if scenario == 'nominal':
                if year in [2037, 2038, 2046, 2050, 2051, 2055, 2061, 2062, 2065, 2070]:
                    mtow_guess = 2000.0  # kg
                elif year in [2031, 2034, 2049, 2058, 2059, 2064]:
                    mtow_guess = 1500.0  # kg
                elif year in [2032]:
                    mtow_guess = 1200.0  # kg
                else:
                    mtow_guess = 1000.0  # kg

            if scenario == 'conservative':
                if year in [2063, 2068]:
                    mtow_guess = 1500.0  # kg
                else:
                    mtow_guess = 1000.0  # kg

            # Standard vehicle
            design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if k != 0:
                vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}
                vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}

            # Standard mission
            mission = StandardMissionProfile(mission_range, cruise_speed)

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = {'scenario': scenario, 'year': year, 'battery_energy_density': battery_energy_density}
                results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=True))
                results_df = pd.DataFrame(results, index=[k])
                results_df.to_csv('optimal_results_by_scenario_by_year/energy_opt_liftcruise_results.csv', mode='a', header=True if k == 0 else False)

            if k == 0:
                f_non_hub_non_wing = results['Aero|Cruise|f_total_non_hub_non_wing']
                Cd0_wing = results['Aero|Cruise|Cd0_wing']

if plot_utopian_data:

    metrics = ['weight', 'energy']
    configs = ['multirotor', 'liftcruise']

    fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True)

    for i, metric in enumerate(metrics):
        for j, config in enumerate(configs):
            k = 2 * i + j
            utopian_data = pd.read_csv(f'optimal_results_by_scenario_by_year/{metric}_opt_{config}_results.csv')
            utopian_data = utopian_data[utopian_data['success'] is True]
            if metric == 'weight':
                metric_tag = 'Weight|takeoff'
            elif metric == 'energy':
                metric_tag = 'Energy|entire_mission'
            axes[i, j].plot(utopian_data[utopian_data['scenario'] == 'conservative']['year'], utopian_data[utopian_data['scenario'] == 'conservative'][metric_tag], label='Conservative' if i == 0 and j == 0 else None)
            axes[i, j].plot(utopian_data[utopian_data['scenario'] == 'nominal']['year'], utopian_data[utopian_data['scenario'] == 'nominal'][metric_tag], label='Nominal' if i == 0 and j == 0 else None)
            axes[i, j].plot(utopian_data[utopian_data['scenario'] == 'aggresive']['year'], utopian_data[utopian_data['scenario'] == 'aggresive'][metric_tag], label='Aggresive' if i == 0 and j == 0 else None)
            if i == 1:
                axes[i, j].set_xlabel('Year')

    axes[0, 0].set_ylabel(r'Takeoff weight $[kg]$')
    axes[1, 0].set_ylabel(r'Required energy $[kWh]$')
    axes[0, 0].set_title('Weight-minimal multirotor', size=11)
    axes[0, 1].set_title('Weight-minimal lift+cruise', size=11)
    axes[1, 0].set_title('Energy-minimal multirotor', size=11)
    axes[1, 1].set_title('Energy-minimal lift+cruise', size=11)
    fig.legend(loc='upper center', bbox_to_anchor=(0.49, 0.95), ncols=3)
    fig.suptitle('Utopian data for different vehicle configurations and design objectives', size=13)
    plt.tight_layout(rect=(0, 0, 1, 0.95))
    plt.show()
