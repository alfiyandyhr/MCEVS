from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
import numpy as np
import sys

# Mission requirement
mission_range = 96000  # m
# cruise_speed_list = np.arange(90.0, 310.0, 10) # km/h
cruise_speed_list = [10, 50]
payload_per_pax = 100.0  # kg

# Hover climb config
hover_climb_speed = 500 * 0.3048 / 60  # m/s; 500 ft/min
hover_climb_distance = 1000 * 0.3048  # m; 1000 ft

# Hover descent config
hover_descent_speed = 300 * 0.3048 / 60  # m/s; 300 ft/min
hover_descent_distance = 1000 * 0.3048  # m; 1000 ft

# No credit climb/descent
no_credit_distance = (6500 - 6000) * 0.3048  # m; 500 ft

# List of cruise powers
cruise_power_list = np.zeros_like(cruise_speed_list)

for i, cruise_speed in enumerate(cruise_speed_list):
    print(f'Solving power for cruise speed = {cruise_speed} km/h')
    sys.stdout.flush()  # To flush the above print output

    # Take-off from 5000 ft ASL
    mission = Mission(planet='Earth', takeoff_altitude=5000 * 0.3048, n_repetition=1)
    mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=hover_climb_speed, distance=hover_climb_distance, n_discrete=10)
    mission.add_segment(name='No Credit Climb', kind='NoCreditClimb', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
    mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed * 1000 / 3600, distance=mission_range, AoA=5.0, n_discrete=10)
    mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
    mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
    # plot_mission_parameters(mission, print_info=False)

    # Design and operation variables
    design_var = {'wing_area': 20.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
    operation_var = {'RPM_lift_rotor': {'hover_climb': 500.0}, 'RPM_propeller': {'cruise': 370.0}}

    # Technology factors
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

    vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
    # vehicle.print_info()

    # Fidelity
    fidelity = {'aero': 1, 'hover_climb': 0}
    if fidelity['hover_climb'] == 0:
        vehicle.weight.max_takeoff = 2142.96575588
    elif fidelity['hover_climb'] == 1:
        vehicle.weight.max_takeoff = 2318.37756911
        vehicle.lift_rotor.global_twist = 27.593661190971346
        vehicle.lift_rotor.RPM['hover_climb'] = 567.0922900053763

    # Analysis
    analysis = PowerAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity)
    results = analysis.evaluate(record=True)

    results_dict = {'total_cruise_power': results.get_val('Power|segment_3', 'kW')[0]}

    # cruise_power_list[i] =

# plt.plot(cruise_speed_list, cruise_power_list, '-o')
# plt.xlabel('Cruise speed [km/h]')
# plt.ylabel('Cruise power [kW]')
# plt.grid()
# plt.show()
