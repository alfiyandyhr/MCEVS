from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
import numpy as np
import pandas as pd
import sys
import warnings
import time

run_without_speed_as_design_var_one_opt = True
run_with_speed_as_design_var_one_opt = False

run_without_speed_as_design_var_all_opt = False
run_with_speed_as_design_var_all_opt = False

rerun_without_speed_as_design_var_all_opt = False

battery_energy_density = 250  # [250,400,550]

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 	# km
speed_i, speed_f, d_speed = 80, 330, 10 	# km/h
range_array = np.arange(range_i, range_f, d_range)
speed_array = np.arange(speed_i, speed_f, d_speed)

print(range_array, len(range_array))
print(speed_array, len(speed_array))

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'},
            'weight_model': {'structure': 'Roskam'}}

if run_without_speed_as_design_var_one_opt or run_with_speed_as_design_var_one_opt:

    range_ij = 100  # km
    speed_ij = 150 * 1.609344  # 150 miles/hour = 241.402 km/h

    if run_without_speed_as_design_var_one_opt:
        mtow_guess = 3000.0
    if run_with_speed_as_design_var_one_opt:
        mtow_guess = 1000.0

    # Standard vehicle
    design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
    operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
    vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

    # Changed battery density
    vehicle.battery.density = float(battery_energy_density)

    # Standard mission
    mission_ij = StandardMissionProfile(range_ij * 1000, speed_ij * 1000 / 3600)

    # Standard optimization
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if run_without_speed_as_design_var_one_opt:
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=True)
            print(results)
        if run_with_speed_as_design_var_one_opt:
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'energy', mtow_guess, speed_as_design_var=True, print=True)
            print(results)

# Expensive simulations, run once !!!
if run_without_speed_as_design_var_all_opt:
    t1 = time.time()
    mtow_guess = 3000.0
    iter_idx = 0
    for i, cruise_speed in enumerate(speed_array):
        f_total_non_hub = None
        for j, mission_range in enumerate(range_array):

            iter_idx += 1
            print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}, f_total_non_hub= {f_total_non_hub}")
            sys.stdout.flush()  # To flush the above print output

            # Standard vehicle
            design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if f_total_non_hub is not None:
                vehicle.f_total_non_hub = {'climb': None, 'cruise': f_total_non_hub, 'descent': None}  # noqa: F821

            # Standard mission
            mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

            # Different initial guesses
            if battery_energy_density == 250:
                if mission_range == 50 and cruise_speed == 90:
                    mtow_guess = 1500.0
                if mission_range == 60 and cruise_speed == 100:
                    mtow_guess = 2000.0
                if mission_range == 30 and cruise_speed in [140, 280, 290]:
                    mtow_guess = 1500.0
                if mission_range == 70 and cruise_speed == 140:
                    mtow_guess = 2500.0
                if mission_range == 40 and cruise_speed == 170:
                    mtow_guess = 1500.0
                if mission_range == 130 and cruise_speed == 200:
                    mtow_guess = 1500.0
                if mission_range == 70 and cruise_speed == 230:
                    mtow_guess = 1500.0
                if mission_range == 150 and cruise_speed == 270:
                    mtow_guess = 1500.0
                if mission_range == 80 and cruise_speed == 300:
                    mtow_guess = 1500.0
                if mission_range == 40 and cruise_speed == 300:
                    mtow_guess = 2500.0
            elif battery_energy_density == 400:
                if mission_range == 40 and cruise_speed == 200:
                    mtow_guess = 1500.0
                if mission_range in [110, 160] and cruise_speed == 260:
                    mtow_guess = 1500.0
                if mission_range in [160, 180, 210] and cruise_speed == 300:
                    mtow_guess = 1500.0
            elif battery_energy_density == 550:
                if mission_range == 130 and cruise_speed == 80:
                    mtow_guess = 1500.0
                if mission_range == 140 and cruise_speed == 80:
                    mtow_guess = 4000.0
                if mission_range == 170 and cruise_speed == 90:
                    mtow_guess = 4500.0
                if mission_range in [180, 190, 200] and cruise_speed == 100:
                    mtow_guess = 1500.0
                if mission_range == 80 and cruise_speed == 130:
                    mtow_guess = 1500.0
                if mission_range == 20 and cruise_speed == 150:
                    mtow_guess = 1500.0
                if mission_range == 70 and cruise_speed == 170:
                    mtow_guess = 1500.0
                if mission_range == 130 and cruise_speed == 190:
                    mtow_guess = 1500.0
                if mission_range in [210, 220] and cruise_speed == 230:
                    mtow_guess = 1500.0
                if mission_range == 190 and cruise_speed == 300:
                    mtow_guess = 1500.0

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=False)
                # print(results)

            if f_total_non_hub is None:
                f_total_non_hub = results['Aero|Cruise|f_total_non_hub']

            results_df = pd.DataFrame(results, index=[iter_idx])

            results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv', mode='a', header=True if iter_idx == 1 else False)

            # Change back to the default guess
            mtow_guess = 3000.0

    t2 = time.time()
    np.savetxt(f'battery_{battery_energy_density}_Whpkg/computational_time.dat', np.array([t2 - t1]))
    print(t2 - t1)

# Expensive simulations, run once !!!
if rerun_without_speed_as_design_var_all_opt:
    mtow_guess = 3000.0
    iter_idx = 0
    for i, cruise_speed in enumerate(speed_array):
        f_total_non_hub = None
        for j, mission_range in enumerate(range_array):

            iter_idx += 1

            if mission_range in [60] and cruise_speed == 100:

                print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
                sys.stdout.flush()  # To flush the above print output

                # Standard vehicle
                design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
                operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
                tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
                vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

                # Changed battery density
                vehicle.battery.density = float(battery_energy_density)

                # Fixed assumed parasite drag from the ref vehicle
                if f_total_non_hub is not None:
                    vehicle.f_total_non_hub = {'climb': None, 'cruise': f_total_non_hub, 'descent': None}  # noqa: F821

                # Standard mission
                mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

                # Different initial guesses
                if battery_energy_density == 250:
                    if mission_range == 50 and cruise_speed == 90:
                        mtow_guess = 1500.0
                    if mission_range == 60 and cruise_speed == 100:
                        mtow_guess = 2000.0
                    if mission_range == 30 and cruise_speed in [140, 280, 290]:
                        mtow_guess = 1500.0
                    if mission_range == 70 and cruise_speed == 140:
                        mtow_guess = 2500.0
                    if mission_range == 40 and cruise_speed == 170:
                        mtow_guess = 1500.0
                    if mission_range == 130 and cruise_speed == 200:
                        mtow_guess = 1500.0
                    if mission_range == 70 and cruise_speed == 230:
                        mtow_guess = 1500.0
                    if mission_range == 150 and cruise_speed == 270:
                        mtow_guess = 1500.0
                    if mission_range == 80 and cruise_speed == 300:
                        mtow_guess = 1500.0
                    if mission_range == 40 and cruise_speed == 300:
                        mtow_guess = 2500.0
                elif battery_energy_density == 400:
                    if mission_range == 40 and cruise_speed == 200:
                        mtow_guess = 1500.0
                    if mission_range in [110, 160] and cruise_speed == 260:
                        mtow_guess = 1500.0
                    if mission_range in [160, 180, 210] and cruise_speed == 300:
                        mtow_guess = 1500.0
                elif battery_energy_density == 550:
                    if mission_range == 130 and cruise_speed == 80:
                        mtow_guess = 1500.0
                    if mission_range == 140 and cruise_speed == 80:
                        mtow_guess = 4000.0
                    if mission_range == 170 and cruise_speed == 90:
                        mtow_guess = 4500.0
                    if mission_range in [180, 190, 200] and cruise_speed == 100:
                        mtow_guess = 1500.0
                    if mission_range == 80 and cruise_speed == 130:
                        mtow_guess = 1500.0
                    if mission_range == 20 and cruise_speed == 150:
                        mtow_guess = 1500.0
                    if mission_range == 70 and cruise_speed == 170:
                        mtow_guess = 1500.0
                    if mission_range == 130 and cruise_speed == 190:
                        mtow_guess = 1500.0
                    if mission_range in [210, 220] and cruise_speed == 230:
                        mtow_guess = 1500.0
                    if mission_range == 190 and cruise_speed == 300:
                        mtow_guess = 1500.0

                # Standard optimization
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=False)
                    # print(results)

                if f_total_non_hub is None:
                    f_total_non_hub = results['Aero|Cruise|f_total_non_hub']

                results['Unnamed: 0'] = iter_idx
                results_df = pd.DataFrame(results, index=[iter_idx])
                last_column = results_df.columns[-1]
                results_df = results_df[[last_column] + [col for col in results_df if col != last_column]]

                results_df_ori = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
                results_df_ori.iloc[iter_idx - 1] = results_df.iloc[0]

                results_df_ori.to_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv', index=False)

                # Change back to the default guess
                mtow_guess = 3000.0

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:

    for i, mission_range in enumerate(range_array):

        sys.stdout.flush()  # To flush the above print output

        cruise_speed_guess = 150 * 1.609344  # 150 miles/hour = 241.402 km/h

        if battery_energy_density == 250:
            if mission_range in [90]:
                mtow_guess = 1500.0
            elif mission_range in [150]:
                mtow_guess = 900.0
            else:
                mtow_guess = 1000.0
        elif battery_energy_density == 400:
            if mission_range in [180]:
                mtow_guess = 900.0
            else:
                mtow_guess = 1000.0
        elif battery_energy_density == 550:
            mtow_guess = 1000.0

        # Standard vehicle
        design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
        operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
        tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
        vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

        # Changed battery density
        vehicle.battery.density = float(battery_energy_density)

        # Fixed assumed parasite drag from the ref vehicle
        if i != 0:
            vehicle.f_total_non_hub = {'climb': None, 'cruise': f_total_non_hub, 'descent': None}  # noqa: F821

        print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

        # Standard mission
        mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed_guess * 1000 / 3600)

        # Standard optimization
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'energy', mtow_guess, speed_as_design_var=True, print=False)
            # print(results)

        if i == 0:
            f_total_non_hub = results['Aero|Cruise|f_total_non_hub']

        results_df = pd.DataFrame(results, index=[i])

        results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv', mode='a', header=True if i == 0 else False)
