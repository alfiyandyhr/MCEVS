import numpy as np
from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Analyses.Energy.Analysis import EnergyAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments

# Mission requirement
mission_range = 96000 # m
cruise_speed = 50.0 # m/s
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
# plot_mission_parameters(mission, print_info=False, save_fig=False)

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=mission.takeoff_altitude)

# Design variables (aircraft design and operation)
design_var1 = {'r_lift_rotor': 1.5, 'cruise_speed': cruise_speed, 'rotor_advance_ratio': 0.3}
design_var2 = {'wing_area': 16.2, 'wing_aspect_ratio': 7.32,
			  'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			  'cruise_speed': cruise_speed, 'propeller_advance_ratio': 1.0}

vehicle1 = StandardMultirotorEVTOL(design_var1, mtow=1250.87074979)
vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, mtow=1415.21244)

# vehicle1.print_info()

# Analysis
analysis = EnergyAnalysis(vehicle=vehicle2, mission=mission, constants=constants, fidelity={'aero':1, 'hover_climb':1})
results = analysis.evaluate(record=True)

# plot_performance_by_segments(mission=mission, vehicle=vehicle1)

print(f"Power|segment_1 = {results.get_val('Power|segment_1', 'kW')[0]} kW")
print(f"Power|segment_2 = {results.get_val('Power|segment_2', 'kW')[0]} kW")
print(f"Power|segment_3 = {results.get_val('Power|segment_3', 'kW')[0]} kW")
print(f"Power|segment_4 = {results.get_val('Power|segment_4', 'kW')[0]} kW")
print(f"Power|segment_5 = {results.get_val('Power|segment_5', 'kW')[0]} kW")
print(f"Energy|entire_mission = {results.get_val('Energy|entire_mission', 'kW*h')[0]} kWh")





