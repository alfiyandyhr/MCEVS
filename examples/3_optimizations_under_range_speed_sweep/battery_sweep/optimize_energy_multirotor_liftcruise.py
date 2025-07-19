from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
import numpy as np
import pandas as pd
import sys
import warnings

run_battery_sweep_multirotor = False
run_battery_sweep_liftcruise = True

battery_energy_density_list = np.arange(250, 560, 10)  # [250,...,550]

# Fixed mission requirement
mission_range = 60  # km
cruise_speed = 200  # km/h

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'},
            'weight_model': {'structure': 'Roskam'},
            'stability': {'AoA_trim': {'cruise': 'ManualFixedValue'}}}

# Expensive simulations, run once !!!
if run_battery_sweep_multirotor:

    data = pd.read_csv('../selection_by_energy/multirotor/battery_250_Whpkg/results_without_speed_as_design_var.csv')
    f_non_hub = data[(data['cruise_speed'] == cruise_speed) & (data['mission_range'] == mission_range)]['Aero|Cruise|f_total_non_hub'].to_numpy()[0]

    mtow_guess = 3000.0
    for i, battery_energy_density in enumerate(battery_energy_density_list):

        print(f"Battery-sweep multirotor; Iter= {i}, Battery= {battery_energy_density} Wh/kg")
        sys.stdout.flush()  # To flush the above print output

        # Standard vehicle
        design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
        operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
        tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
        vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

        # Changed battery density
        vehicle.battery.density = float(battery_energy_density)

        # Fixed assumed parasite drag from the ref vehicle
        vehicle.f_total_non_hub = {'climb': None, 'cruise': f_non_hub, 'descent': None}

        # Standard mission
        mission = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

        # Different initial guesses
        if battery_energy_density in [430, 500]:
            mtow_guess = 1500.0  # kg

        # Standard optimization
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            results = {'battery_energy_density': battery_energy_density}
            results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=False))

        results_df = pd.DataFrame(results, index=[i])

        results_df.to_csv('energy_battery_sweep_multirotor.csv', mode='a', header=True if i == 0 else False)

        # Change back to the default guess
        mtow_guess = 3000.0

# Expensive simulations, run once !!!
if run_battery_sweep_liftcruise:

    data = pd.read_csv('../selection_by_energy/liftcruise/battery_250_Whpkg/results_without_speed_as_design_var.csv')
    f_non_hub_non_wing = data[(data['cruise_speed'] == cruise_speed) & (data['mission_range'] == mission_range)]['Aero|Cruise|f_total_non_hub_non_wing'].to_numpy()[0]
    Cd0_wing = data[(data['cruise_speed'] == cruise_speed) & (data['mission_range'] == mission_range)]['Aero|Cruise|Cd0_wing'].to_numpy()[0]

    mtow_guess = 3000.0
    for i, battery_energy_density in enumerate(battery_energy_density_list):
        # if battery_energy_density == 490:
        print(f"Battery-sweep liftcruise; Iter= {i}, Battery= {battery_energy_density} Wh/kg")
        sys.stdout.flush()  # To flush the above print output

        # Standard vehicle
        design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
        operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 500.0}}
        tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
        vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

        # Changed battery density
        vehicle.battery.density = float(battery_energy_density)

        # Fixed assumed parasite drag from the ref vehicle
        vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}
        vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}

        # Standard mission
        mission = StandardMissionProfile(mission_range * 1000, cruise_speed * 1000 / 3600)

        # Different initial guesses
        if battery_energy_density in [320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 540, 550]:
            mtow_guess = 1000.0  # kg

        if battery_energy_density in [310, 490]:
            mtow_guess = 1050.0

        if battery_energy_density in [270]:
            mtow_guess = 800.0

        # Standard optimization
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            results = {'battery_energy_density': battery_energy_density}
            results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=False))

        results_df = pd.DataFrame(results, index=[i])

        results_df.to_csv('energy_battery_sweep_liftcruise.csv', mode='a', header=True if i == 0 else False)
        # print(results)
        # Change back to the default guess
        mtow_guess = 3000.0
