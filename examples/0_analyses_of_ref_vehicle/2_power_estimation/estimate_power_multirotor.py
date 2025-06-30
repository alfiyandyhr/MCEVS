from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
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
design_var1 = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
operation_var1 = {'RPM_lift_rotor': {'hover_climb': None, 'cruise': 450.0}}
design_var2 = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
operation_var2 = {'RPM_lift_rotor': {'hover_climb': None}, 'RPM_propeller': {'cruise': 500.0}}

# Technology factors
tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

vehicle = StandardMultirotorEVTOL(design_var1, operation_var1, tfs, n_pax=6, payload_per_pax=payload_per_pax)
# vehicle.print_info()

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'}
            # 'power_model': {'hover_climb': 'ModifiedMomentumTheory'}
            # 'power_model': {'hover_climb': 'BladeElementMomentumTheory'}
            }

if fidelity['power_model']['hover_climb'] == 'MomentumTheory':
    vehicle.weight.max_takeoff = 4121.13997362
    vehicle.lift_rotor.RPM['hover_climb'] = 400.0
elif fidelity['power_model']['hover_climb'] == 'ModifiedMomentumTheory':
    vehicle.weight.max_takeoff = 4112.30957032
    vehicle.lift_rotor.RPM['hover_climb'] = 400.0
elif fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
    vehicle.weight.max_takeoff = 3920.99011566
    vehicle.lift_rotor.global_twist = 15.43370966
    vehicle.lift_rotor.RPM['hover_climb'] = 289.78701964

# Run analysis
analysis = PowerAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity)
results = analysis.evaluate(record=True)

print('\n--- Sanity Check: Segment Powers ---')
print('Power segment_1 = ', results.get_val('Power|segment_1', 'kW'))
print('Power segment_2 = ', results.get_val('Power|segment_2', 'kW'))
print('Power segment_3 = ', results.get_val('Power|segment_3', 'kW'))
print('Power segment_4 = ', results.get_val('Power|segment_4', 'kW'))
print('Power segment_5 = ', results.get_val('Power|segment_5', 'kW'))
print('FM = ', results.get_val('LiftRotor|HoverClimb|FM'))

# plot_performance_by_segments(mission=mission, vehicle=vehicle)

# Note:
# Analyses to check:
#   fidelity['power_model']['hover_climb'] in ['MomentumTheory', 'ModifiedMomentumTheory', 'BladeElementMomentumTheory']
