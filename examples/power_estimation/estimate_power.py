import numpy as np
from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments

# Mission requirement
mission_range = 96000 # m
cruise_speed = 50.0 # m/s
payload_per_pax = 100.0 # kg

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
mission.add_segment('Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, AoA=5.0, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=no_credit_distance, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=hover_descent_speed, distance=hover_descent_distance, n_discrete=10)
# plot_mission_parameters(mission, print_info=False)

# Design and operation variables
design_var1 = {'r_lift_rotor': 4.0}
operation_var1 = {'RPM_lift_rotor': {'hover_climb':500.0,'cruise':450.0}, 'cruise_speed': cruise_speed}
design_var2 = {'wing_area': 20.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
operation_var2 = {'RPM_lift_rotor': {'hover_climb':500.0}, 'RPM_propeller': {'cruise':370.0}, 'cruise_speed': cruise_speed}

# Technology factors
tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}

vehicle1 = StandardMultirotorEVTOL(design_var1, operation_var1, tfs, n_pax=6, payload_per_pax=payload_per_pax)
vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, operation_var2, tfs, n_pax=6, payload_per_pax=payload_per_pax)
# vehicle1.print_info()

# Fidelity
fidelity = {'aero':1, 'hover_climb':1}
if fidelity['hover_climb'] == 0:
	vehicle1.weight.max_takeoff = 2141.99321998
	vehicle2.weight.max_takeoff = 2142.96575588
elif fidelity['hover_climb'] == 1:
	vehicle1.weight.max_takeoff = 2199.6029094
	vehicle1.lift_rotor.global_twist = 7.8645777892370585
	vehicle1.lift_rotor.RPM['hover_climb'] = 354.76421553294784
	vehicle2.weight.max_takeoff = 2318.37756911
	vehicle2.lift_rotor.global_twist = 27.593661190971346
	vehicle2.lift_rotor.RPM['hover_climb'] = 567.0922900053763

# Analysis (change vehicle=vehicle1 or vehicle=vehicle2)
analysis = PowerAnalysis(vehicle=vehicle2, mission=mission, fidelity=fidelity)
results = analysis.evaluate(record=True)

plot_performance_by_segments(mission=mission, vehicle=vehicle1)

# Note:
# Analyses to check:
# 	fidelity['hover_climb'] in [0, 1]
# 	vehicle=vehicle1 or vehicle=vehicle2
