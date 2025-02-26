from MCEVS.Vehicles.Container import MultirotorEVTOL, LiftPlusCruiseEVTOL
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Analyses.Energy.Analysis import EnergyAnalysis
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments
from MCEVS.Missions.Container import Mission
import numpy as np

create_vehicle1 = True
create_vehicle2 = False

# Mission requirement
mission_range = 69450 # m
if create_vehicle1: cruise_speed = 50.4156 # m/s
if create_vehicle2: cruise_speed = 57.6178 # m/s

# Cruise range
climb_dist_X   = np.sqrt(cruise_speed**2 - 4.572**2) * (((10000-6050)*0.3048)/4.572)
cruise_range   = mission_range - climb_dist_X

# Take-off from 6000 ft ASL
mission = Mission(planet='Earth', takeoff_altitude=6000*0.3048, n_repetition=2)
mission.add_segment(name='Taxi 1', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=100*0.3048/60, distance=50*0.3048, n_discrete=10)
mission.add_segment(name='Transition 1', kind='ConstantPower', duration=10.0, percent_max_power=100.0)
mission.add_segment(name='Constant Climb', kind='ClimbConstantVyConstantVx', distance_Y=(10000-6050)*0.3048, speed_Y=900*0.3048/60, speed=cruise_speed, n_discrete=10)
mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=cruise_range, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=(10000-6050)*0.3048, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Transition 2', kind='ConstantPower', duration=10.0, percent_max_power=100.0)
mission.add_segment(name='Hover', kind='ConstantPower', duration=30.0, percent_max_power=100.0)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=100*0.3048/60, distance=50*0.3048, n_discrete=10)
mission.add_segment(name='Taxi 2', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Reserve Cruise', kind='ReserveCruise', duration=20*60, percent_max_power=10.0)
# plot_mission_p arameters(mission, print_info=True)

if create_vehicle1:
	vehicle1 = MultirotorEVTOL(tf_structure=1.4, tf_propulsion=1.0, tf_equipment=1.4, mtow=2939.27616)
	vehicle1.add_component(kind='battery', density=420.0, efficiency=1.0, max_discharge=1.0)
	vehicle1.add_component(kind='fuselage', length=21.0*0.3048, max_diameter=6.7455*0.3048, number_of_passenger=6, payload_per_pax=91.474387)
	vehicle1.add_component(kind='landing_gear', gear_type='skid', skid_length=9.36364*0.3048, skid_heights=[2.46136*0.3048, 2.78871*0.3048])
	t_per_c_1		= (0.2+0.2+0.36)/3
	t_per_c_2		= (0.2+0.2+0.36)/3
	t_per_c_3		= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_4		= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_list 	= [t_per_c_1, t_per_c_2, t_per_c_3, t_per_c_4]
	span1 			= (6.46552+5.97448)*0.3048
	span2 			= (2.49015+2.02370+8.10000)*0.3048
	span_list 		= [span1, span1, span2, span2]
	sweep_list 		= [-45.0, -45.0, (46.68595+46.68595+40.0)/3, (46.68595+46.68595+40.0)/3]
	vehicle1.add_component(kind='boom', represented_as='wing', thickness_to_chord_ratio=t_per_c_list,
						  number_of_booms=4, span_list=span_list, sweep_list=sweep_list)
	r_lift_rotor 	= 13.1 * 0.3048
	r_to_R_list 	= [0.175, 0.285, 0.395, 0.505, 0.615, 0.725, 0.835, 0.945]
	c_to_R_list 	= [0.071475, 0.069425, 0.067375, 0.065325, 0.063275, 0.061225, 0.059175, 0.057125]
	pitch_list 		= 4.0 + np.array([6.90, 5.58, 4.26, 2.94, 1.62, 0.30, -1.02, -2.34])
	c_lift_rotor 	= np.sum(c_to_R_list)/8 * r_lift_rotor
	vehicle1.add_component(kind='lift_rotor', n_rotor=4, n_blade=3, n_section=8, airfoil='BOEING_VERTOL_VR12',
						  radius=r_lift_rotor, hub_radius=0.12*r_lift_rotor, chord=c_lift_rotor,
						  r_to_R_list=r_to_R_list, c_to_R_list=c_to_R_list, pitch_list=pitch_list,
						  Cd0=0.0089, figure_of_merit=0.75, advance_ratio=0.24)
	# vehicle1.print_info()

	# Analysis
	analysis1 = PowerAnalysis(vehicle=vehicle1, mission=mission, fidelity={'aero':1, 'hover_climb':1})
	results = analysis1.evaluate(record=True)

	# MTOW = results.get_val('Weight|takeoff')[0]
	# OEW = MTOW - results.get_val('Weight|payload')[0]
	# W_batt = results.get_val('Weight|battery')[0]
	# W_empty = OEW - W_batt

	# E_req = results.get_val('Energy|entire_mission', 'kW*h')[0]
	P_hover = results.get_val('Power|segment_2', 'kW')[0]
	P_cruise = results.get_val('Power|segment_5', 'kW')[0]

	hover_tip_speed = results.get_val('LiftRotor|hover_climb_rpm')[0]*2*np.pi/60*r_lift_rotor
	hover_FM = results.get_val('LiftRotor|hover_climb_FM')[0]
	hover_DL = results.get_val('DiskLoading|LiftRotor|segment_2', 'Pa')[0]

	# cruise_f_total = results.get_val('Aero|Cruise|f_total','m**2')[0]
	# cruise_f_fuselage = results.get_val('Aero|Cruise|f_fuselage','m**2')[0]
	# cruise_f_rotor_hub = results.get_val('Aero|Cruise|f_rotor_hub','m**2')[0]
	# cruise_drag = results.get_val('Aero|Cruise|total_drag', 'N')[0]

	# print(f"MTOW: published= 2939.27616; calculated= {MTOW}; Difference= {np.round((MTOW-2939.27616)/2939.27616*100, 2)}%")
	# print(f'OEW: published= 2394.96576; calculated= {OEW}; Difference= {np.round((OEW-2394.96576)/2394.96576*100, 2)}%')
	# print(f'Battery weight: published= 710.77866; calculated= {W_batt}; Difference= {np.round((W_batt-710.77866)/710.77866*100, 2)}%')
	# print(f'Empty weight: published= 1684.187096; calculated= {W_empty}; Difference= {np.round((W_empty-1684.187096)/1684.187096*100, 2)}%')
	# print('---------------------------------------------------------------------------------------------')
	# print(f'Required energy: published= 297.22246; calculated= {E_req}; Difference= {np.round((E_req-297.22246)/297.22246*100, 2)}%')
	print(f'P_hover: published= 345.1525714; calculated= {P_hover}; Difference= {np.round((P_hover-345.1525714)/345.1525714*100, 2)}%')
	print(f'P_cruise: published= 262.616087; calculated= {P_cruise}; Difference= {np.round((P_cruise-262.616087)/262.616087*100, 2)}%')
	print('---------------------------------------------------------------------------------------------')
	print(f'Hover tip speed: published= 167.64; calculated= {hover_tip_speed}; Difference= {np.round((hover_tip_speed-167.64)/167.64*100, 2)}%')
	print(f'Hover FM: published= 0.7; calculated= {hover_FM}; Difference= {np.round((hover_FM-0.7)/0.7*100, 2)}%')
	print(f'Hover DL: published= 143.6408; calculated= {hover_DL}; Difference= {np.round((hover_DL-143.6408)/143.6408*100, 2)}%')
	# print('---------------------------------------------------------------------------------------------')
	# print(f'EFPA fuselage: published= 0.130064256; calculated= {cruise_f_fuselage}; Difference= {np.round((cruise_f_fuselage-0.130064256)/0.130064256*100, 2)}%')
	# print(f'EFPA rotor hubs: published= 0.901159488; calculated= {cruise_f_rotor_hub}; Difference= {np.round((cruise_f_rotor_hub-0.901159488)/0.901159488*100, 2)}%')
	# print(f'Total EFPA: published= 1.033308276; calculated= {cruise_f_total}; Difference= {np.round((cruise_f_total-1.033308276)/1.033308276*100, 2)}%')
	# print(f'Total cruise drag: published= 1188.38997; calculated= {cruise_drag}; Difference= {np.round((cruise_drag-1188.38997)/1188.38997*100, 2)}%')
	# print('---------------------------------------------------------------------------------------------')
	# print('Parasite drag coeff non hubs = ', vehicle1.Cd0)
	# print(f"LiftRotor|hover_climb_rpm = {results.get_val('LiftRotor|hover_climb_rpm')[0]}")
	# print(f"LiftRotor|hover_climb|thrust_residual_square = {results.get_val('LiftRotor|hover_climb|thrust_residual_square')[0]}")
	# print('Weight|residual = ', results.get_val('Weight|residual')[0])

if create_vehicle2:
	vehicle2 = LiftPlusCruiseEVTOL(tf_structure=1.1, tf_propulsion=1.0, tf_equipment=1.1, mtow=3723.99032)
	vehicle2.add_component(kind='battery', density=400.0, efficiency=1.0, max_discharge=1.0)
	vehicle2.add_component(kind='fuselage', length=30.0*0.3048, max_diameter=6.12707*0.3048, number_of_passenger=6, payload_per_pax=91.474387)
	vehicle2.add_component(kind='wing', area=210.27814*0.3048**2, aspect_ratio=12.12761, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0)
	vehicle2.add_component(kind='horizontal_tail', area=39.51120*0.3048**2, aspect_ratio=4.30363, taper_ratio=0.6, max_root_thickness=0.15*3.78750*0.3048, thickness_to_chord_ratio=0.15)
	vehicle2.add_component(kind='vertical_tail', area=27.34325*0.3048**2, aspect_ratio=1.17990, max_root_thickness=0.11500*9.73719*0.3048, sweep_angle=35, thickness_to_chord_ratio=(0.115+0.14+0.15+0.15+0.115)/5)
	vehicle2.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3048, ultimate_load_factor=5.7)
	
	r_lift_rotor 	= 5.0*0.3048
	r_to_R_list1 	= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list1 	= [0.548288, 0.530863, 0.513438, 0.496013, 0.478588, 0.461163, 0.443738, 0.426313]
	pitch_list1		= 8.0 + np.array([7.359375, 5.878125, 4.396875, 2.915625, 1.434375, -0.046875, -1.528125, -3.009375])
	c_lift_rotor 	= np.sum(c_to_R_list1)/8 * r_lift_rotor
	
	r_propeller 	= 4.5*0.3048
	r_to_R_list2 	= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list2 	= [0.500656, 0.484769, 0.468881, 0.452994, 0.437106, 0.421219, 0.405331, 0.389444]
	pitch_list2		= 8.0 + np.array([17.591875, 14.135625, 10.679375, 7.223125, 3.766875, 0.310625, -3.145625, -6.601875])
	c_propeller 	= np.sum(c_to_R_list2)/8 * r_propeller

	vehicle2.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, n_section=8, airfoil='BOEING_VERTOL_VR12',
						   radius=r_lift_rotor, hub_radius=0.21*r_lift_rotor, chord=c_lift_rotor, r_to_R_list=r_to_R_list1, c_to_R_list=c_to_R_list1,
						   pitch_list=pitch_list1, hub_length=0.3048, hub_max_diameter=2.25*0.3048, figure_of_merit=0.75)
	vehicle2.add_component(kind='propeller', n_propeller=1, n_blade=3, n_section=8, airfoil='BOEING_VERTOL_VR12',
						   radius=r_propeller, hub_radius=0.21*r_propeller, chord=c_propeller, r_to_R_list=r_to_R_list2, c_to_R_list=c_to_R_list2,
						   pitch_list=pitch_list2, hub_length=0.3048, hub_max_diameter=1.5*0.3048, figure_of_merit=0.75, Cd0=0.0089, advance_ratio=0.6)

	vehicle2.add_component(kind='boom', represented_as='fuselage', length=16.0*0.3048, max_diameter=0.3048, number_of_booms=4)
	# vehicle2.print_info()

	# Analysis
	analysis2 = PowerAnalysis(vehicle=vehicle2, mission=mission, fidelity={'aero':1, 'hover_climb':1})
	results = analysis2.evaluate(record=True)

	# MTOW = results.get_val('Weight|takeoff')[0]
	# OEW = MTOW - results.get_val('Weight|payload')[0]
	# W_batt = results.get_val('Weight|battery')[0]
	# W_empty = OEW - W_batt

	# E_req = results.get_val('Energy|entire_mission', 'kW*h')[0]
	P_hover = results.get_val('Power|segment_2', 'kW')[0]
	P_cruise = results.get_val('Power|segment_5', 'kW')[0]

	hover_tip_speed = results.get_val('LiftRotor|hover_climb_rpm')[0]*2*np.pi/60*r_lift_rotor
	hover_FM = results.get_val('LiftRotor|hover_climb_FM')[0]
	hover_DL = results.get_val('DiskLoading|LiftRotor|segment_2', 'Pa')[0]

	# cruise_f_total = results.get_val('Aero|Cruise|f_total','m**2')[0]
	# cruise_f_fuselage = results.get_val('Aero|Cruise|f_fuselage','m**2')[0]
	# cruise_f_rotor_hub = results.get_val('Aero|Cruise|f_rotor_hub','m**2')[0]
	# cruise_drag = results.get_val('Aero|Cruise|total_drag', 'N')[0]

	# print(f"MTOW: published= 3723.99032; calculated= {MTOW}; Difference= {np.round((MTOW-3723.99032)/3723.99032*100, 2)}%")
	# print(f'OEW: published= 3175.597592; calculated= {OEW}; Difference= {np.round((OEW-3175.597592)/3175.597592*100, 2)}%')
	# print(f'Battery weight: published= 767.931256; calculated= {W_batt}; Difference= {np.round((W_batt-767.931256)/767.931256*100, 2)}%')
	# print(f'Empty weight: published= 2407.666336; calculated= {W_empty}; Difference= {np.round((W_empty-2407.666336)/2407.666336*100, 2)}%')
	print('---------------------------------------------------------------------------------------------')
	# print(f'Required energy: published= 309.166914; calculated= {E_req}; Difference= {np.round((E_req-309.166914)/309.166914*100, 2)}%')
	print(f'P_hover: published= 827.3239189; calculated= {P_hover}; Difference= {np.round((P_hover-827.3239189)/827.3239189*100, 2)}%')
	print(f'P_cruise: published= 245.8713655; calculated= {P_cruise}; Difference= {np.round((P_cruise-245.8713655)/245.8713655*100, 2)}%')
	print('---------------------------------------------------------------------------------------------')
	print(f'Hover tip speed: published= 178.308; calculated= {hover_tip_speed}; Difference= {np.round((hover_tip_speed-178.308)/178.308*100, 2)}%')
	print(f'Hover FM: published= 0.74; calculated= {hover_FM}; Difference= {np.round((hover_FM-0.74)/0.74*100, 2)}%')
	print(f'Hover DL: published= 625.2721616; calculated= {hover_DL}; Difference= {np.round((hover_DL-625.2721616)/625.2721616*100, 2)}%')
	# print('---------------------------------------------------------------------------------------------')
	# print(f'EFPA fuselage: published= 0.157935168; calculated= {cruise_f_fuselage}; Difference= {np.round((cruise_f_fuselage-0.157935168)/0.157935168*100, 2)}%')
	# print(f'EFPA rotor hubs: published= 0.111483648; calculated= {cruise_f_rotor_hub}; Difference= {np.round((cruise_f_rotor_hub-0.111483648)/0.111483648*100, 2)}%')
	# print(f'Total EFPA: published= 1.550157718; calculated= {cruise_f_total}; Difference= {np.round((cruise_f_total-1.550157718)/1.550157718*100, 2)}%')
	# print(f'Total cruise drag: published= 2328.565293; calculated= {cruise_drag}; Difference= {np.round((cruise_drag-2328.565293)/2328.565293*100, 2)}%')
	# print('---------------------------------------------------------------------------------------------')
	# print('Parasite drag coeff non hubs = ', vehicle2.Cd0)
	# print(f"LiftRotor|hover_climb_rpm = {results.get_val('LiftRotor|hover_climb_rpm')[0]}")
	# print(f"LiftRotor|hover_climb|thrust_residual_square = {results.get_val('LiftRotor|hover_climb|thrust_residual_square')[0]}")
	# print('Weight|residual = ', results.get_val('Weight|residual')[0])

