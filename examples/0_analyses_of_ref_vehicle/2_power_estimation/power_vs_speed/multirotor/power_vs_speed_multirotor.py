from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
import numpy as np
import pandas as pd
import sys

# Mission requirement
mission_range = 96000  # m
cruise_speed_list = np.arange(10.0, 310.0, 10)  # km/h
# cruise_speed_list = [10, 50]
payload_per_pax = 100.0  # kg

# Hover climb config
hover_climb_speed = 500 * 0.3048 / 60  # m/s; 500 ft/min
hover_climb_distance = 1000 * 0.3048  # m; 1000 ft

# Hover descent config
hover_descent_speed = 300 * 0.3048 / 60  # m/s; 300 ft/min
hover_descent_distance = 1000 * 0.3048  # m; 1000 ft

# No credit climb/descent
no_credit_distance = (6500 - 6000) * 0.3048  # m; 500 ft

for i, cruise_speed in enumerate(cruise_speed_list):
    print(f'Solving power for cruise speed = {cruise_speed} km/h')
    sys.stdout.flush()  # To flush the above print output

    # Take-off from 5000 ft ASL
    mission = Mission(planet='Earth', takeoff_altitude=5000 * 0.3048, n_repetition=1)
    mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=hover_climb_speed, distance=hover_climb_distance, n_discrete=10)
    mission.add_segment(name='No Credit Climb', kind='NoCreditClimb', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
    mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed * 1000 / 3600, distance=mission_range, n_discrete=10)
    mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
    mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
    # plot_mission_parameters(mission, print_info=False)

    # Design and operation variables
    design_var = {'r_lift_rotor': 4.0}
    operation_var = {'RPM_lift_rotor': {'hover_climb': 500.0, 'cruise': 450.0}}

    # Technology factors
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

    vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
    # vehicle.print_info()

    # Fidelity
    fidelity = {'aero': 1, 'hover_climb': 0}
    if fidelity['hover_climb'] == 0:
        vehicle.weight.max_takeoff = 2141.99321998
    elif fidelity['hover_climb'] == 1:
        vehicle.weight.max_takeoff = 2199.6029094
        vehicle.lift_rotor.global_twist = 7.8645777892370585
        vehicle.lift_rotor.RPM['hover_climb'] = 354.76421553294784

    # Analysis
    analysis = PowerAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity)
    results = analysis.evaluate(record=False)

    results_dict = {'cruise_speed': cruise_speed,
                    'cruise_power': results.get_val('Power|segment_3', 'kW')[0],
                    'profile_power': results.get_val('Power|segment_3|profile_power', 'kW')[0],
                    'induced_power': results.get_val('Power|segment_3|induced_power', 'kW')[0],
                    'propulsive_power': results.get_val('Power|segment_3|propulsive_power', 'kW')[0]}

    results_df = pd.DataFrame(results_dict, index=[i])

    results_df.to_csv('multirotor_P_vs_v.csv', mode='a', header=True if i == 0 else False)
