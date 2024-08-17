from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Optimization.Container import DesignProblem
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments
import numpy as np

# Mission requirement
mission_range = 30000 # m
cruise_speed = 50.0 # m/s
payload_weight = 400.0

# Cruise range
climb_dist_X   = np.sqrt(cruise_speed**2 - 2.54**2) * (304.8/2.54)
descent_dist_X = np.sqrt(cruise_speed**2 - 1.524**2) * (304.8/1.524)
cruise_range   = mission_range - climb_dist_X - descent_dist_X

# Take-off from 5000 ft ASL or 1500 m
mission = Mission(takeoff_altitude=0.0)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=152.4, n_discrete=10)
mission.add_segment(name='Constant Climb', kind='ClimbConstantVyConstantVx', distance_Y=304.8, speed_Y=2.54, speed=cruise_speed, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=cruise_range, n_discrete=10)
mission.add_segment(name='Constant Descent', kind='DescentConstantVyConstantVx', distance_Y=304.8, speed_Y=1.524, speed=cruise_speed, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=152.4, n_discrete=10)
# plot_mission_parameters(mission, print_info=False)


# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=mission.takeoff_altitude)

# Initial design variables (aircraft design and operation)
design_var = {'wing_area': 8.0, 'wing_aspect_ratio': 10.0,
			  'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			  'cruise_speed': cruise_speed, 'propeller_advance_ratio': 1.0}

vehicle = StandardLiftPlusCruiseEVTOL(design_var)

# # Analysis
# analysis = WeightAnalysis(vehicle=vehicle, mission=mission, constants=constants, sizing_mode=True)

# Design problem
problem = DesignProblem(vehicle=vehicle,
						mission=mission,
						constants=constants,
						algorithm='gradient-based')

problem.add_objective('Weight|takeoff')
problem.add_design_var('Mission|cruise_speed', 40.0, 70.0, 'm/s')
problem.add_design_var('Wing|area', 6.0, 20.0, 'm**2')
problem.add_design_var('Wing|aspect_ratio', 6.0, 12.0)
problem.add_design_var('LiftRotor|radius', 0.90, 1.75, 'm')
problem.add_design_var('Propeller|radius', 0.70, 1.75, 'm')
problem.add_design_var('Propeller|advance_ratio', 0.01, 1.3)
problem.add_constraint('Aero|Cruise|CL', 0.0, 0.6)
problem.add_constraint('Propeller|Cruise|thrust_coefficient', 0.0, 0.14*vehicle.propeller.solidity)
problem.add_constraint('DiskLoading|LiftRotor|segment_1', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|Propeller|segment_2', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|Propeller|segment_3', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|Propeller|segment_4', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|Propeller|segment_5', 0.0, 600.0, 'N/m**2')
# 759.3522839629419
# Optimization
res = problem.run_optimization()

# Optimal design var
print('v_cruise =', res.get_val('Mission|cruise_speed', 'm/s'), 'm/s')
print("AR_wing =", res.get_val('Wing|aspect_ratio'))
print("S_wing =", res.get_val('Wing|area', 'm**2'), 'm**2')
print('r_rotor =', res.get_val('LiftRotor|radius', 'm'), 'm')
print('r_propeller =', res.get_val('Propeller|radius', 'm'), 'm')
print('J_propeller =', res.get_val('Propeller|advance_ratio'))
print('MTOW =', res.get_val('Weight|takeoff', 'kg'), 'kg')

# Plot optimal design performance
plot_performance_by_segments(mission=mission, vehicle=vehicle)







