from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
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
            'weight_model': {'structure': 'Roskam'},
            'stability': {'AoA_trim': {'cruise': 'ManualFixedValue'}}}

if run_without_speed_as_design_var_one_opt or run_with_speed_as_design_var_one_opt:

    range_ij = 100
    speed_ij = 150 * 1.609344  # 150 miles/hour = 241.402 km/h

    if run_without_speed_as_design_var_one_opt:
        mtow_guess = 3000.0
    if run_with_speed_as_design_var_one_opt:
        mtow_guess = 1000.0

    # Standard vehicle
    design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
    operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
    vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

    # Changed battery density
    vehicle.battery.density = float(battery_energy_density)

    # Standard mission
    mission_ij = StandardMissionProfile(range_ij * 1000, speed_ij * 1000 / 3600)

    # Standard optimization
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if run_without_speed_as_design_var_one_opt:
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=True)
            print(results)
        if run_with_speed_as_design_var_one_opt:
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=True, print=True)
            print(results)

# Expensive simulations, run once !!!
if run_without_speed_as_design_var_all_opt:
    t1 = time.time()
    mtow_guess = 3000.0
    iter_idx = 0
    for i, cruise_speed in enumerate(speed_array):
        f_total_non_hub_non_wing = None
        Cd0_wing = None
        for j, mission_range in enumerate(range_array):

            iter_idx += 1
            print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}, f_total_non_hub_non_wing= {f_total_non_hub_non_wing}, Cd0_wing= {Cd0_wing}")
            sys.stdout.flush()  # To flush the above print output

            # Standard vehicle
            design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
            operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
            tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
            vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

            # Changed battery density
            vehicle.battery.density = float(battery_energy_density)

            # Fixed assumed parasite drag from the ref vehicle
            if f_total_non_hub_non_wing is not None and Cd0_wing is not None:
                vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_total_non_hub_non_wing, 'descent': None}  # noqa: F821
                vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}  # noqa: F821

            # Different initial guesses
            if battery_energy_density == 250:
                if mission_range == 40 and cruise_speed == 130:
                    mtow_guess = 1500.0
                if mission_range == 60 and cruise_speed == 140:
                    mtow_guess = 1500.0
                if mission_range in [70, 80, 90, 100] and cruise_speed == 150:
                    mtow_guess = 1500.0
                if mission_range in [80, 90, 130, 150] and cruise_speed == 160:
                    mtow_guess = 1500.0
                if mission_range in [130, 140, 150, 180] and cruise_speed == 170:
                    mtow_guess = 1500.0
                if mission_range in [150, 190, 210, 220] and cruise_speed == 180:
                    mtow_guess = 1500.0
                if mission_range in [170, 180, 190, 200, 210, 220] and cruise_speed == 190:
                    mtow_guess = 1500.0
                if mission_range in [190, 200, 210] and cruise_speed == 200:
                    mtow_guess = 1500.0
                if mission_range in [110] and cruise_speed == 250:
                    mtow_guess = 1500.0
            elif battery_energy_density == 400:
                if mission_range in [120, 130] and cruise_speed == 130:
                    mtow_guess = 1500.0
                if mission_range in [150, 160] and cruise_speed == 140:
                    mtow_guess = 1500.0
                if mission_range in [180, 190, 200, 210, 220] and cruise_speed == 150:
                    mtow_guess = 1500.0
            elif battery_energy_density == 550:
                if mission_range in [150] and cruise_speed == 120:
                    mtow_guess = 1500.0
                if mission_range in [190, 200, 210] and cruise_speed == 130:
                    mtow_guess = 1500.0
                if mission_range in [10] and cruise_speed == 160:
                    mtow_guess = 1500.0
                if mission_range in [20, 30] and cruise_speed == 170:
                    mtow_guess = 1500.0
                if mission_range in [10, 30, 40] and cruise_speed == 180:
                    mtow_guess = 1500.0
                if mission_range in [30, 40] and cruise_speed == 190:
                    mtow_guess = 1500.0
                if mission_range in [20, 50] and cruise_speed == 200:
                    mtow_guess = 1500.0
                if mission_range in [40, 50, 60] and cruise_speed == 210:
                    mtow_guess = 1500.0
                if mission_range in [60, 70, 80] and cruise_speed == 220:
                    mtow_guess = 1500.0
                if mission_range in [60, 70, 80, 90] and cruise_speed == 230:
                    mtow_guess = 1500.0
                if mission_range in [60] and cruise_speed == 240:
                    mtow_guess = 1500.0
                if mission_range in [70, 80, 100] and cruise_speed == 250:
                    mtow_guess = 1500.0
                if mission_range == 90 and cruise_speed in [270, 280]:
                    mtow_guess = 1500.0
                if mission_range in [50, 80] and cruise_speed == 290:
                    mtow_guess = 1500.0
                if mission_range in [60, 80, 90] and cruise_speed == 300:
                    mtow_guess = 1500.0
                if mission_range in [70, 80] and cruise_speed == 310:
                    mtow_guess = 1500.0
                if mission_range in [10, 120] and cruise_speed == 320:
                    mtow_guess = 1500.0

            # Standard mission
            mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

            # Standard optimization
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=False)
                # print(results)

            if f_total_non_hub_non_wing is None and Cd0_wing is None:
                f_total_non_hub_non_wing = results['Aero|Cruise|f_total_non_hub_non_wing']
                Cd0_wing = results['Aero|Cruise|Cd0_wing']

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
        f_total_non_hub_non_wing = None
        Cd0_wing = None
        for j, mission_range in enumerate(range_array):
            iter_idx += 1
            if mission_range in [10, 120] and cruise_speed == 320:

                print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}, f_total_non_hub_non_wing= {f_total_non_hub_non_wing}, Cd0_wing= {Cd0_wing}")
                sys.stdout.flush()  # To flush the above print output

                # Standard vehicle
                design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
                operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
                tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
                vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

                # Changed battery density
                vehicle.battery.density = float(battery_energy_density)

                # Fixed assumed parasite drag from the ref vehicle
                if f_total_non_hub_non_wing is not None and Cd0_wing is not None:
                    vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_total_non_hub_non_wing, 'descent': None}  # noqa: F821
                    vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}  # noqa: F821

                # Different initial guesses
                if battery_energy_density == 250:
                    if mission_range == 40 and cruise_speed == 130:
                        mtow_guess = 1500.0
                    if mission_range == 60 and cruise_speed == 140:
                        mtow_guess = 1500.0
                    if mission_range in [70, 80, 90, 100] and cruise_speed == 150:
                        mtow_guess = 1500.0
                    if mission_range in [80, 90, 130, 150] and cruise_speed == 160:
                        mtow_guess = 1500.0
                    if mission_range in [130, 140, 150, 180] and cruise_speed == 170:
                        mtow_guess = 1500.0
                    if mission_range in [150, 190, 210, 220] and cruise_speed == 180:
                        mtow_guess = 1500.0
                    if mission_range in [170, 180, 190, 200, 210, 220] and cruise_speed == 190:
                        mtow_guess = 1500.0
                    if mission_range in [190, 200, 210] and cruise_speed == 200:
                        mtow_guess = 1500.0
                    if mission_range in [110] and cruise_speed == 250:
                        mtow_guess = 1500.0
                elif battery_energy_density == 400:
                    if mission_range in [120, 130] and cruise_speed == 130:
                        mtow_guess = 1500.0
                    if mission_range in [150, 160] and cruise_speed == 140:
                        mtow_guess = 1500.0
                    if mission_range in [180, 190, 200, 210, 220] and cruise_speed == 150:
                        mtow_guess = 1500.0
                elif battery_energy_density == 550:
                    if mission_range in [150] and cruise_speed == 120:
                        mtow_guess = 1500.0
                    if mission_range in [190, 200, 210] and cruise_speed == 130:
                        mtow_guess = 1500.0
                    if mission_range in [10] and cruise_speed == 160:
                        mtow_guess = 1500.0
                    if mission_range in [20, 30] and cruise_speed == 170:
                        mtow_guess = 1500.0
                    if mission_range in [10, 30, 40] and cruise_speed == 180:
                        mtow_guess = 1500.0
                    if mission_range in [30, 40] and cruise_speed == 190:
                        mtow_guess = 1500.0
                    if mission_range in [20, 50] and cruise_speed == 200:
                        mtow_guess = 1500.0
                    if mission_range in [40, 50, 60] and cruise_speed == 210:
                        mtow_guess = 1500.0
                    if mission_range in [60, 70, 80] and cruise_speed == 220:
                        mtow_guess = 1500.0
                    if mission_range in [60, 70, 80, 90] and cruise_speed == 230:
                        mtow_guess = 1500.0
                    if mission_range in [60] and cruise_speed == 240:
                        mtow_guess = 1500.0
                    if mission_range in [70, 80, 100] and cruise_speed == 250:
                        mtow_guess = 1500.0
                    if mission_range == 90 and cruise_speed in [270, 280]:
                        mtow_guess = 1500.0
                    if mission_range in [50, 80] and cruise_speed == 290:
                        mtow_guess = 1500.0
                    if mission_range in [60, 80, 90] and cruise_speed == 300:
                        mtow_guess = 1500.0
                    if mission_range in [70, 80] and cruise_speed == 310:
                        mtow_guess = 1500.0
                    if mission_range in [10, 120] and cruise_speed == 320:
                        mtow_guess = 1500.0

                # Standard mission
                mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

                # Standard optimization
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=False)
                    # print(results)

                if f_total_non_hub_non_wing is None and Cd0_wing is None:
                    f_total_non_hub_non_wing = results['Aero|Cruise|f_total_non_hub_non_wing']
                    Cd0_wing = results['Aero|Cruise|Cd0_wing']

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
            mtow_guess = 1000.0
        elif battery_energy_density == 400:
            mtow_guess = 1000.0
        elif battery_energy_density == 550:
            mtow_guess = 1000.0

        # Standard vehicle
        design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
        operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
        tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
        vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

        # Changed battery density
        vehicle.battery.density = float(battery_energy_density)

        # Fixed assumed parasite drag from the ref vehicle
        if i != 0:
            vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}  # noqa: F821
            vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}  # noqa: F821

        print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

        # Standard mission
        mission_ij = StandardMissionProfile(mission_range * 1000, cruise_speed_guess * 1000 / 3600)

        # Standard optimization
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=True, print=False)
            # print(results)

        if i == 0:
            f_non_hub_non_wing = results['Aero|Cruise|f_total_non_hub_non_wing']
            Cd0_wing = results['Aero|Cruise|Cd0_wing']

        results_df = pd.DataFrame(results, index=[i])

        results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv', mode='a', header=True if i == 0 else False)
        # results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_test.csv', mode='a', header=True if i == 0 else False)
