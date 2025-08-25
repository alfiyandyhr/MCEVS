from MCEVS.Missions.Container import Mission
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
import numpy as np

# Mission requirement
mission_range = 37.5 * 1852  # nmi to m
cruise_speed = 83 * 0.514444  # knot to m/s
payload_per_pax = (1208 / 6) * 0.453592  # lb to kg

# Cruise range
climb_dist_X = np.sqrt(cruise_speed**2 - 4.572**2) * (((10000 - 6050) * 0.3048) / 4.572)
cruise_range = mission_range - climb_dist_X

# Hover climb config
hover_climb_speed = 100 * 0.3048 / 60  # m/s; 100 ft/min
hover_climb_distance = 50 * 0.3048  # m; 50 ft

# Hover descent config
hover_descent_speed = 100 * 0.3048 / 60  # m/s; 100 ft/min
hover_descent_distance = 50 * 0.3048  # m; 50 ft

# No credit climb/descent
no_credit_distance = (10000 - 6050) * 0.3048  # m; 3950 ft

# Take-off from 6000 ft ASL
mission = Mission(planet='Earth', takeoff_altitude=6000 * 0.3048, n_repetition=2)
mission.add_segment(name='Taxi 1', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=hover_climb_speed, distance=hover_climb_distance, n_discrete=10)
mission.add_segment(name='Transition 1', kind='HoverStay', duration=10)
mission.add_segment(name='No Credit Climb', kind='NoCreditClimb', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=mission_range, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Transition 2', kind='HoverStay', duration=10)
mission.add_segment(name='Hover', kind='HoverStay', duration=30)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
mission.add_segment(name='Taxi 2', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Reserve Cruise', kind='ReserveCruise', duration=20 * 60)
# plot_mission_parameters(mission, print_info=False)
# mission.print_info()

# Design and operation variables
design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
operation_var = {'RPM_lift_rotor': {'hover_climb': None}, 'RPM_propeller': {'cruise': 500.0}}

# Technology factors
tfs = {'tf_structure': 1.0, 'tf_propulsion': 1.0, 'tf_equipment': 1.0}

# Vehicle creation
vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
vehicle.battery.density = 400.0  # Wh/kg
vehicle.battery.max_discharge = 1.0
vehicle.battery.efficiency = 1.0  # Propulsion efficiency
vehicle.lift_rotor.figure_of_merit = 0.75

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'},
            'weight_model': {'structure': 'Roskam'},
            'stability': {'AoA_trim': {'cruise': 'ManualFixedValue'}}}

# Assumptions
if fidelity['power_model']['hover_climb'] == 'MomentumTheory':
    vehicle.lift_rotor.RPM['hover_climb'] = 1050.422591  # assumed from the published data

"""
BEST OF Claude-Sonnet-4
--- Sanity Checks ---
Weight|residual =  98.62129022835998

--- Validation: Component Weights ---
DGW (lb): published= 9482; calculated= 8624.914349734669; Difference= -9.04%
OEW (lb): published= 8274; calculated= 7416.915335112796; Difference= -10.36%
Battery weight (lb): published= 2058; calculated= 1842.115790826351; Difference= -10.49%
Empty weight (lb): published= 6216; calculated= 5574.799544286445; Difference= -10.32%

--- Validation: Performance ---
Disk loading (lb/ft2): published= 15.1; calculated= 13.7190674175524; Difference= -9.15%
Tip speed (ft/s): published= 550; calculated= 550.0000001083922; Difference= 0.0%
L_by_D_cruise: published= 7.9; calculated= 6.747636558750007; Difference= -14.59%
Available power lift rotor (hp): published= 8x189; calculated= 8x230.38770693902296; Difference= -13.37%
Available power propeller (hp): published= 1x838; calculated= 1x488.518277144183; Difference= 6.49%
"""
# vehicle.tf_structure = 1.1707
# vehicle.tf_equipment = 1.2107
# vehicle.propeller.RPM['cruise'] = 1490.0
# vehicle.tf_propulsion = 0.9607
# vehicle.lift_rotor.motor_power_margin = 6.6
# vehicle.propeller.motor_power_margin = 174.0

"""
BEST OF ChatGPT
--- Sanity Checks ---
Weight|residual =  28.667809817455105

--- Validation: Component Weights ---
DGW (lb): published= 9482; calculated= 8632.380240686418; Difference= -8.96%
OEW (lb): published= 8274; calculated= 7424.381226064545; Difference= -10.27%
Battery weight (lb): published= 2058; calculated= 1851.54055415887; Difference= -10.03%
Empty weight (lb): published= 6216; calculated= 5572.840671905676; Difference= -10.35%

--- Validation: Performance ---
Disk loading (lb/ft2): published= 15.1; calculated= 13.730942904907504; Difference= -9.07%
Tip speed (ft/s): published= 550; calculated= 550.0000001083922; Difference= 0.0%
L_by_D_cruise: published= 7.9; calculated= 6.716226167082627; Difference= -14.98%
Available power lift rotor (hp): published= 8x189; calculated= 8x230.68691367896145; Difference= -13.26%
Available power propeller (hp): published= 1x838; calculated= 1x491.2278238816481; Difference= 7.08%
"""
vehicle.tf_structure = 1.16
vehicle.tf_equipment = 1.23
vehicle.tf_propulsion = 0.96
vehicle.propeller.RPM['cruise'] = 1500.0
vehicle.lift_rotor.motor_power_margin = 6.6
vehicle.propeller.motor_power_margin = 174.0

# Analysis
# vehicle.print_info()
analysis = WeightAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
results = analysis.evaluate(record=False, weight_guess=2500.0)

# plot_performance_by_segments(mission=mission, vehicle=vehicle)

DGW = results.get_val('Weight|takeoff', 'lb')[0]
OEW = DGW - results.get_val('Weight|payload', 'lb')[0]
W_batt = results.get_val('Weight|battery', 'lb')[0]
W_empty = OEW - W_batt
E_req = results.get_val('Energy|entire_mission', 'kW*h')[0]
P_hover = results.get_val('Power|segment_8', 'hp')[0]
P_cruise = results.get_val('Power|segment_5', 'W')[0]
P_cruise_hp = results.get_val('Power|segment_5', 'hp')[0]
DL_hover = results.get_val('DiskLoading|LiftRotor|segment_2', 'lbf/ft**2')[0]
v_tip = vehicle.lift_rotor.RPM['hover_climb'] * 2 * np.pi * vehicle.lift_rotor.radius / 60 * 3.28084  # ft/s
L_by_D_cruise = (DGW * 0.453592) * 9.81 * cruise_speed / P_cruise

print('--- Sanity Checks ---')
print('Weight|residual = ', results.get_val('Weight|residual')[0])

print('\n--- Validation: Component Weights ---')

print(f'DGW (lb): published= 9482; calculated= {DGW}; Difference= {np.round((DGW-9482)/9482*100, 2)}%')
print(f'OEW (lb): published= 8274; calculated= {OEW}; Difference= {np.round((OEW-8274)/8274*100, 2)}%')
print(f'Battery weight (lb): published= 2058; calculated= {W_batt}; Difference= {np.round((W_batt-2058)/2058*100, 2)}%')
print(f'Empty weight (lb): published= 6216; calculated= {W_empty}; Difference= {np.round((W_empty-6216)/6216*100, 2)}%')

print('\n--- Validation: Performance ---')
print(f'Disk loading (lb/ft2): published= 15.1; calculated= {DL_hover}; Difference= {np.round((DL_hover-15.1)/15.1*100, 2)}%')
print(f'Tip speed (ft/s): published= 550; calculated= {v_tip}; Difference= {np.round((v_tip-550)/550*100, 2)}%')
print(f'L_by_D_cruise: published= 7.9; calculated= {L_by_D_cruise}; Difference= {np.round((L_by_D_cruise-7.9)/7.9*100, 2)}%')
print(f'Available power lift rotor (hp): published= 8x189; calculated= 8x{P_hover/8*(1+vehicle.lift_rotor.motor_power_margin/100)}; Difference= {np.round((P_hover/8*(1+vehicle.lift_rotor.motor_power_margin/100)-189)/189*100, 2)}%')
print(f'Available power propeller (hp): published= 1x838; calculated= 1x{P_cruise_hp*(1+vehicle.propeller.motor_power_margin/100)}; Difference= {np.round((P_cruise_hp*(1+vehicle.propeller.motor_power_margin/100)-838)/838*100, 2)}%')
