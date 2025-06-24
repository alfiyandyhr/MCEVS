from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
# from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments

# Mission requirement
mission_range = 60 * 1609.344  # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s
payload_per_pax = 100.0  # kg

# Hover climb config
hover_climb_speed = 500 * 0.3048 / 60  # m/s; 500 ft/min
hover_climb_distance = 1000 * 0.3048  # m; 1000 ft

# Hover descent config
hover_descent_speed = 300 * 0.3048 / 60  # m/s; 300 ft/min
hover_descent_distance = 1000 * 0.3048  # m; 1000 ft

# No credit climb/descent
no_credit_distance = (6500 - 6000) * 0.3048  # m; 500 ft

# Take-off from 5000 ft ASL
mission = Mission(planet='Earth', takeoff_altitude=5000 * 0.3048, n_repetition=1)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=hover_climb_speed, distance=hover_climb_distance, n_discrete=10)
mission.add_segment(name='No Credit Climb', kind='NoCreditClimb', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, AoA=5.0, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
mission.add_segment(name='Reserve Cruise', kind='ReserveCruise', duration=20 * 60)
# plot_mission_parameters(mission, print_info=False)

# Design and operation variables
design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
operation_var = {'RPM_lift_rotor': {'hover_climb': None}, 'RPM_propeller': {'cruise': 500.0}}

# Technology factors
tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

# Vehicle creation
vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
# vehicle.print_info()

# Solver fidelity
fidelity = {'aero': 1, 'hover_climb': 2}
if fidelity['hover_climb'] == 0:
    vehicle.lift_rotor.RPM['hover_climb'] = 400.0
elif fidelity['hover_climb'] == 1:
    vehicle.lift_rotor.RPM['hover_climb'] = 700.0
elif fidelity['hover_climb'] == 2:
    vehicle.lift_rotor.RPM['hover_climb'] = 500.0
    vehicle.lift_rotor.motor_power_margin = 0.0
    vehicle.propeller.motor_power_margin = 0.0

# Analysis
analysis = WeightAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity, sizing_mode=True, solved_by='optimization')
results = analysis.evaluate(record=True, mtow_guess=2500.0)

# plot_performance_by_segments(mission=mission, vehicle=vehicle)

print('--- Sanity Check: Aero Drag Coeff ---')
print('f_total_non_hub_non_wing =', [vehicle.f_total_non_hub_non_wing['cruise']])
print('Cd0 wing only =', [vehicle.wing.Cd0['cruise']])
print('\n--- Sanity Check: Segment Powers ---')
print('Power segment_1 = ', results.get_val('Power|segment_1', 'kW'))
print('Power segment_2 = ', results.get_val('Power|segment_2', 'kW'))
print('Power segment_3 = ', results.get_val('Power|segment_3', 'kW'))
print('Power segment_4 = ', results.get_val('Power|segment_4', 'kW'))
print('Power segment_5 = ', results.get_val('Power|segment_5', 'kW'))
print('Required energy = ', results.get_val('Energy|entire_mission', 'kW*h'))
# print(results.get_val('Power|Propeller|maximum'))
# print(results.get_val('Power|LiftRotor|maximum'))
print('\n--- Sanity Check: Component Weights ---')
print('Payload weight = ', results.get_val('Weight|payload'))
print('Battery weight = ', results.get_val('Weight|battery'))
print('Propulsion weight = ', results.get_val('Weight|propulsion'))
print('Structure weight = ', results.get_val('Weight|structure'))
print('Equipment weight = ', results.get_val('Weight|equipment'))
print('MTOW = ', results.get_val('Weight|takeoff'))
print('\n--- Sanity Check: Residuals and others---')
print(f"LiftRotor|global_twist = {results.get_val('LiftRotor|global_twist')[0]}")
print(f"LiftRotor|HoverClimb|RPM = {results.get_val('LiftRotor|HoverClimb|RPM')[0]}")
print(f"LiftRotor|HoverClimb|FM = {results.get_val('LiftRotor|HoverClimb|FM')[0]}")
if fidelity['hover_climb'] == 2:
    print(f"LiftRotor|HoverClimb|thrust_residual_square = {results.get_val('LiftRotor|HoverClimb|thrust_residual_square')[0]}")
print('Weight|residual = ', results.get_val('Weight|residual')[0])
