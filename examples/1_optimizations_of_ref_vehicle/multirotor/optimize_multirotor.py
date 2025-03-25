import numpy as np
from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Optimization.Container import DesignProblem
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments

# Mission requirement
mission_range = 96000 # m
cruise_speed = 50.0 # m/s
# mission_range = 150*1000 # m
# cruise_speed = 144*1000/3600 # m/s
payload_per_pax = 100.0 # kg
print(f'Duration = {mission_range/cruise_speed/3600} hours')

# Hover climb config
hover_climb_speed = 500*0.3048/60 # m/s; 500 ft/min
hover_climb_distance = 1000*0.3048 # m; 1000 ft

# Hover descent config
hover_descent_speed = 300*0.3048/60 # m/s; 300 ft/min
hover_descent_distance = 1000*0.3048 # m; 1000 ft

# No credit climb/descent
no_credit_distance = (6500-6000)*0.3048 # m; 500 ft

# Take-off from 5000 ft ASL
mission = Mission(planet='Earth', takeoff_altitude=5000*0.3048, n_repetition=1)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=hover_climb_speed, distance=hover_climb_distance, n_discrete=10)
mission.add_segment(name='No Credit Climb', kind='NoCreditClimb', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
# plot_mission_parameters(mission, print_info=False)

# Design and operation variables
design_var = {'r_lift_rotor': 4.0}
operation_var = {'RPM': {'cruise':450.0}}

# Technology factors
tfs = {'tf_structure':1.0, 'tf_propulsion':1.0, 'tf_equipment':1.0}

# Vehicle creation
vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=payload_per_pax)
# vehicle.print_info()

# Design problem
fidelity = {'aero':1, 'hover_climb':0}
problem = DesignProblem(vehicle=vehicle,
						mission=mission,
						fidelity=fidelity,
						algorithm='gradient-based')

problem.add_objective('Weight|takeoff')
# problem.add_objective('Weight|battery')
# problem.add_objective('Weight|residual')
# problem.add_objective('Energy|entire_mission')

if fidelity['hover_climb'] == 1: problem.add_design_var('Weight|takeoff', 1700.0, 5000.0, 2199.60290727, 'kg')
else: problem.add_design_var('Weight|takeoff', 100.0, 5000.0, 1500, 'kg')
problem.add_design_var('LiftRotor|radius', 1.0, 5.0, 4.0, 'm')
problem.add_design_var('LiftRotor|Cruise|RPM', 100.0, 1500.0, 450.0, 'rpm')

problem.add_constraint('LiftRotor|Cruise|mu', 0.01, 0.5, None)
if fidelity['hover_climb'] == 1: problem.add_constraint('Weight|residual', -0.001, 0.001, 'kg')
else: problem.add_constraint('Weight|residual', 0.0, 0.0, 'kg')
problem.add_constraint('LiftRotor|Cruise|thrust_coefficient', 0.0, 0.14*vehicle.lift_rotor.solidity)
problem.add_constraint('LiftRotor|HoverClimb|T_to_P', 0.0, 12.0)
problem.add_constraint('LiftRotor|Cruise|T_to_P', 0.0, 12.0)
problem.add_constraint('LiftRotor|HoverDescent|T_to_P', 0.0, 12.0)

if fidelity['hover_climb'] == 1:
	problem.add_design_var('LiftRotor|global_twist', 0.0, 20.0, 7.864577821145281, 'deg')
	problem.add_design_var('LiftRotor|HoverClimb|RPM', 10.0, 1500.0, 354.7642145353529, 'rpm')
	problem.add_constraint('LiftRotor|HoverClimb|thrust_residual_square', 0.0, 0.001, None)
	problem.add_constraint('LiftRotor|HoverClimb|FM', 0.6, 0.8, None)
	# problem.add_constraint('LiftRotor|HoverClimb|thrust_coefficient', 0.0, 1.2*vehicle.lift_rotor.solidity)

# Optimization
results = problem.run_optimization()

# Optimal design var
print('--- Sanity Check: Optimal design vars ---')
print('LiftRotor|radius =', results.get_val('LiftRotor|radius', 'm'), 'm')
print('LiftRotor|chord =', results.get_val('LiftRotor|chord', 'm'), 'm')
if fidelity['hover_climb'] == 1: print('LiftRotor|global_twist =', results.get_val('LiftRotor|global_twist', 'deg'), 'deg')
if fidelity['hover_climb'] == 1: print('LiftRotor|HoverClimb|RPM =', results.get_val('LiftRotor|HoverClimb|RPM', 'rpm'), 'rpm')
print('LiftRotor|Cruise|RPM =', results.get_val('LiftRotor|Cruise|RPM', 'rpm'), 'rpm')

print('\n--- Sanity Check: Optimal weight ---')
print('Weight|payload = ', results.get_val('Weight|payload'))
print('Weight|battery = ', results.get_val('Weight|battery'))
print('Weight|propulsion = ', results.get_val('Weight|propulsion'))
print('Weight|structure = ', results.get_val('Weight|structure'))
print('Weight|equipment = ', results.get_val('Weight|equipment'))
print('Weight|takeoff =', results.get_val('Weight|takeoff', 'kg'), 'kg')

print('\n--- Sanity Check: Performance at optimal weight ---')
print(vehicle.Cd0)
print('Power|segment_1 = ', results.get_val('Power|segment_1', 'kW'), 'kW')
print('Power|segment_2 = ', results.get_val('Power|segment_2', 'kW'), 'kW')
print('Power|segment_3 = ', results.get_val('Power|segment_3', 'kW'), 'kW')
print('Power|segment_4 = ', results.get_val('Power|segment_4', 'kW'), 'kW')
print('Power|segment_5 = ', results.get_val('Power|segment_5', 'kW'), 'kW')
print('Energy|entire_mission =', results.get_val('Energy|entire_mission', 'kW*h'), 'kWh')

print('\n--- Sanity Check: Constraints ---')
if fidelity['hover_climb'] == 1: print('CT/sigma HoverClimb = ', results.get_val('LiftRotor|HoverClimb|thrust_coefficient')/vehicle.lift_rotor.solidity)
print('CT/sigma Cruise = ', results.get_val('LiftRotor|Cruise|thrust_coefficient')/vehicle.lift_rotor.solidity)
if fidelity['hover_climb'] == 1: print('LiftRotor|HoverClimb|J = ', results.get_val('LiftRotor|HoverClimb|J'), '')
print('LiftRotor|Cruise|mu = ', results.get_val('LiftRotor|Cruise|mu'), '')
print('DiskLoading|LiftRotor|segment_1 = ', results.get_val('DiskLoading|LiftRotor|segment_1', 'N/m**2'), 'N/m**2')
print('DiskLoading|LiftRotor|segment_2 = ', results.get_val('DiskLoading|LiftRotor|segment_2', 'N/m**2'), 'N/m**2')
print('DiskLoading|LiftRotor|segment_3 = ', results.get_val('DiskLoading|LiftRotor|segment_3', 'N/m**2'), 'N/m**2')
print('DiskLoading|LiftRotor|segment_4 = ', results.get_val('DiskLoading|LiftRotor|segment_4', 'N/m**2'), 'N/m**2')
print('DiskLoading|LiftRotor|segment_5 = ', results.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2'), 'N/m**2')
print('LiftRotor|HoverClimb|T_to_P =', results.get_val('LiftRotor|HoverClimb|T_to_P', 'g/W'), 'g/W')
print('LiftRotor|Cruise|T_to_P =', results.get_val('LiftRotor|Cruise|T_to_P', 'g/W'), 'g/W')
print('LiftRotor|HoverDescent|T_to_P =', results.get_val('LiftRotor|HoverDescent|T_to_P', 'g/W'), 'g/W')
print('LiftRotor|HoverClimb|FM =', results.get_val('LiftRotor|HoverClimb|FM', None), '')
if fidelity['hover_climb'] == 1: print('LiftRotor|HoverClimb|thrust_residual_square =', results.get_val('LiftRotor|HoverClimb|thrust_residual_square', None), '')
print('Weight|residual =', results.get_val('Weight|residual', 'kg'), 'kg')

# plot_performance_by_segments(mission=mission, vehicle=vehicle)

