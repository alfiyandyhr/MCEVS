import numpy as np
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission requirement
mission_range = 30000 # m
cruise_speed = 30.0 # m/s
payload_weight = 400.0

# Cruise range
climb_dist_X   = np.sqrt(cruise_speed**2 - 2.54**2) * (304.8/2.54)
descent_dist_X = np.sqrt(cruise_speed**2 - 1.524**2) * (304.8/1.524)
cruise_range   = mission_range - climb_dist_X - descent_dist_X

# Take-off from 5000 ft ASL or 1500 m
mission = Mission(takeoff_altitude=0.0)
# mission.add_segment('Hover Climb', kind='HoverStay', duration=120.0, n_discrete=5)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=152.4, n_discrete=10)
mission.add_segment(name='Constant Climb', kind='ClimbConstantVyConstantVx', distance_Y=304.8, speed_Y=2.54, speed=cruise_speed, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=cruise_range, n_discrete=10)
mission.add_segment(name='Constant Descent', kind='DescentConstantVyConstantVx', distance_Y=304.8, speed_Y=1.524, speed=cruise_speed, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=152.4, n_discrete=10)
# mission.add_segment('Hover Descent', kind='HoverStay', duration=120.0, n_discrete=5)
# plot_mission_parameters(mission, print_info=False)

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=mission.takeoff_altitude)

# Design variables (aircraft design and operation)
design_var = {'wing_area': 8.0, 'wing_aspect_ratio': 10.0,
			  'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			  'cruise_speed': cruise_speed, 'propeller_advance_ratio': 1.0}

vehicle = StandardLiftPlusCruiseEVTOL(design_var)

# Analysis
analysis = WeightAnalysis(vehicle=vehicle, mission=mission, constants=constants, sizing_mode=True)
results = analysis.evaluate()

print('Power segment_1 = ', results.get_val('Power|segment_1', 'W'))
print('Power segment_2 = ', results.get_val('Power|segment_2', 'W'))
print('Power segment_3 = ', results.get_val('Power|segment_3', 'W'))
print('Power segment_4 = ', results.get_val('Power|segment_4', 'W'))
print('Power segment_5 = ', results.get_val('Power|segment_5', 'W'))
print('Required energy = ', results.get_val('Energy|entire_mission', 'W*h'))
# print(results.get_val('Power|Propeller|maximum'))
# print(results.get_val('Power|LiftRotor|maximum'))

print('Payload weight = ', results.get_val('Weight|payload'))
print('Battery weight = ', results.get_val('Weight|battery'))
print('Propulsion weight = ', results.get_val('Weight|propulsion'))
print('Structure weight = ', results.get_val('Weight|structure'))
print('Equipment weight = ', results.get_val('Weight|equipment'))
print('MTOW = ', results.get_val('Weight|takeoff'))
print('Residual = ', results.get_val('Weight|residual'))





