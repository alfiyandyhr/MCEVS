from MCEVS.Missions.Container import Mission
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments
from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
import numpy as np

# Mission requirement
mission_range = 37.5 * 1852  # nmi to m
cruise_speed = 91 * 0.514444  # knot to m/s
payload_per_pax = (1209 / 6) * 0.453592  # lb to kg

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
mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Transition 2', kind='HoverStay', duration=10)
mission.add_segment(name='Hover', kind='HoverStay', duration=30)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
mission.add_segment(name='Taxi 2', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Reserve Cruise', kind='ReserveCruise', duration=20 * 60)
# plot_mission_parameters(mission, print_info=False)
# mission.print_info()

# Design and operation variables
design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
operation_var = {'RPM_lift_rotor': {'hover_climb': None, 'cruise': 480.0}}

# Technology factors
tfs = {'tf_structure': 1.25, 'tf_propulsion': 1.25, 'tf_equipment': 1.25}

# Vehicle creation
vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
vehicle.battery.density = 400.0  # Wh/kg
vehicle.battery.max_discharge = 1.0
vehicle.battery.efficiency = 1.0  # Propulsion efficiency
vehicle.lift_rotor.figure_of_merit = 0.75

# Solver fidelity
fidelity = {'aerodynamics': {'parasite': 'ComponentBuildUp', 'induced': 'ParabolicDragPolar'},
            'power_model': {'hover_climb': 'MomentumTheory'},
            'weight_model': {'structure': 'Roskam'}}

# Assumptions
if fidelity['power_model']['hover_climb'] == 'MomentumTheory':
    vehicle.lift_rotor.RPM['hover_climb'] = 311.3900960848773  # assumed from the published data
vehicle.lift_rotor.motor_power_margin = 50.0  # %

# Analysis
vehicle.print_info()
analysis = WeightAnalysis(vehicle=vehicle, mission=mission, fidelity=fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
results = analysis.evaluate(record=False, weight_guess=4000.0)

# plot_performance_by_segments(mission=mission, vehicle=vehicle)

DGW = results.get_val('Weight|takeoff', 'lb')[0]
OEW = DGW - results.get_val('Weight|payload', 'lb')[0]
W_batt = results.get_val('Weight|battery', 'lb')[0]
W_empty = OEW - W_batt
E_req = results.get_val('Energy|entire_mission', 'kW*h')[0]
P_hover = results.get_val('Power|segment_8', 'hp')[0]
P_cruise = results.get_val('Power|segment_5', 'W')[0]
DL_hover = results.get_val('DiskLoading|LiftRotor|segment_2', 'lbf/ft**2')[0]
v_tip = vehicle.lift_rotor.RPM['hover_climb'] * 2 * np.pi * vehicle.lift_rotor.radius / 60 * 3.28084  # ft/s
L_by_D_cruise = (DGW * 0.453592) * 9.81 * cruise_speed / P_cruise

print('--- Sanity Checks ---')
print('Weight|residual = ', results.get_val('Weight|residual')[0])

print('\n--- Validation: Component Weights ---')

print(f'DGW (lb): published= 7221; calculated= {DGW}; Difference= {np.round((DGW-7221)/7221*100, 2)}%')
print(f'OEW (lb): published= 6012; calculated= {OEW}; Difference= {np.round((OEW-6012)/6012*100, 2)}%')
print(f'Battery weight (lb): published= 1742; calculated= {W_batt}; Difference= {np.round((W_batt-1742)/1742*100, 2)}%')
print(f'Empty weight (lb): published= 4270; calculated= {W_empty}; Difference= {np.round((W_empty-4270)/4270*100, 2)}%')

print('\n--- Validation: Performance ---')
print(f'Disk loading (lb/ft2): published= 3; calculated= {DL_hover}; Difference= {np.round((DL_hover-3)/3*100, 2)}%')
print(f'Tip speed (ft/s): published= 450; calculated= {v_tip}; Difference= {np.round((v_tip-450)/450*100, 2)}%')
print(f'L_by_D_cruise: published= 5.8; calculated= {L_by_D_cruise}; Difference= {np.round((L_by_D_cruise-5.8)/5.8*100, 2)}%')
print(f'Available power (hp): published= 4x181; calculated= 4x{P_hover/4*(1+50/100)}; Difference= {np.round((P_hover/4*(1+50/100)-181)/181*100, 2)}%')
