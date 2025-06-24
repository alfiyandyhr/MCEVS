from MCEVS.Vehicles.Container import MultirotorEVTOL, LiftPlusCruiseEVTOL
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
# from MCEVS.Utils.Plots import plot_mission_parameters, plot_performance_by_segments
from MCEVS.Missions.Container import Mission
import numpy as np

create_vehicle1 = True
create_vehicle2 = False

# Mission requirement
mission_range = 37.5 * 1852  # nmi to m
payload_per_pax = (1209 / 6) * 0.453592  # lb to kg
if create_vehicle1:
    cruise_speed = 91 / 1.944  # knot to m/s
if create_vehicle2:
    cruise_speed = 83 / 1.944  # knot to m/s

# Cruise range
climb_dist_X = np.sqrt(cruise_speed**2 - 4.572**2) * (((10000 - 6050) * 0.3048) / 4.572)
cruise_range = mission_range - climb_dist_X

# Take-off from 6000 ft ASL
mission = Mission(planet='Earth', takeoff_altitude=6000 * 0.3048, n_repetition=2)
mission.add_segment(name='Taxi 1', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=100 * 0.3048 / 60, distance=50 * 0.3048, n_discrete=10)
mission.add_segment(name='Transition 1', kind='ConstantPower', duration=10.0, percent_max_power=100.0)
mission.add_segment(name='Constant Climb', kind='ClimbConstantVyConstantVx', distance_Y=(10000 - 6050) * 0.3048, speed_Y=900 * 0.3048 / 60, speed=cruise_speed, n_discrete=10)
mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=cruise_range, n_discrete=10)
mission.add_segment(name='No Credit Descent', kind='NoCreditDescent', distance_Y=(10000 - 6050) * 0.3048, distance_X=0.0, n_discrete=10)
mission.add_segment(name='Transition 2', kind='ConstantPower', duration=10.0, percent_max_power=100.0)
mission.add_segment(name='Hover', kind='ConstantPower', duration=30.0, percent_max_power=100.0)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=100 * 0.3048 / 60, distance=50 * 0.3048, n_discrete=10)
mission.add_segment(name='Taxi 2', kind='ConstantPower', duration=15.0, percent_max_power=10.0)
mission.add_segment(name='Reserve Cruise', kind='ReserveCruise', duration=20 * 60)
# plot_mission_parameters(mission, print_info=True)

if create_vehicle1:
    vehicle1 = MultirotorEVTOL(tf_structure=1.0, tf_propulsion=1.0, tf_equipment=1.0)
    vehicle1.add_component(kind='battery', density=420.0, efficiency=1.0, max_discharge=1.0)
    vehicle1.add_component(kind='fuselage', length=21.0 * 0.3048, max_diameter=6.7455 * 0.3048, number_of_passenger=6, payload_per_pax=payload_per_pax)
    vehicle1.add_component(kind='landing_gear', gear_type='skid', skid_length=9.36364 * 0.3048, skid_heights=[2.46136 * 0.3048, 2.78871 * 0.3048])
    t_per_c_1 = (0.2 + 0.2 + 0.36) / 3
    t_per_c_2 = (0.2 + 0.2 + 0.36) / 3
    t_per_c_3 = (0.27273 + 0.27273 + 0.2 + 0.32) / 4
    t_per_c_4 = (0.27273 + 0.27273 + 0.2 + 0.32) / 4
    t_per_c_list = [t_per_c_1, t_per_c_2, t_per_c_3, t_per_c_4]
    span1 = (6.46552 + 5.97448) * 0.3048
    span2 = (2.49015 + 2.02370 + 8.10000) * 0.3048
    span_list = [span1, span1, span2, span2]
    sweep_list = [-45.0, -45.0, (46.68595 + 46.68595 + 40.0) / 3, (46.68595 + 46.68595 + 40.0) / 3]
    vehicle1.add_component(kind='boom', represented_as='wing', thickness_to_chord_ratio=t_per_c_list,
                           number_of_booms=4, span_list=span_list, sweep_list=sweep_list)
    r_lift_rotor = 13.8 * 0.3048
    r_to_R_list = [0.175, 0.285, 0.395, 0.505, 0.615, 0.725, 0.835, 0.945]
    c_to_R_list = [0.071475, 0.069425, 0.067375, 0.065325, 0.063275, 0.061225, 0.059175, 0.057125]
    w_to_R_list = [2 * (0.175 - 0.12), 0.285 - 0.175, 0.395 - 0.285, 0.505 - 0.395, 0.615 - 0.505, 0.725 - 0.615, 0.835 - 0.725, 0.945 - 0.835]
    pitch_list = np.array([6.90, 5.58, 4.26, 2.94, 1.62, 0.30, -1.02, -2.34])
    mean_c_to_R = np.sum(c_to_R_list) / 8
    vehicle1.add_component(kind='lift_rotor', n_rotor=4, n_blade=3, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_QR',
                           radius=r_lift_rotor, hub_radius=0.12 * r_lift_rotor, mean_c_to_R=mean_c_to_R,
                           r_to_R_list=r_to_R_list, c_to_R_list=c_to_R_list, w_to_R_list=w_to_R_list, pitch_list=pitch_list,
                           global_twist=4.1, Cd0=0.0089, figure_of_merit=0.75, RPM={'climb': 300.0, 'cruise': 450.0})
    # vehicle1.print_info()

    # Analysis
    analysis1 = WeightAnalysis(vehicle=vehicle1, mission=mission, fidelity={'aero': 1, 'hover_climb': 1}, sizing_mode=True, solved_by='nonlinear_solver')
    results = analysis1.evaluate(record=True, value_guess={'hover_climb_RPM': 530.0, 'mtow': 3000.0})

    MTOW = results.get_val('Weight|takeoff')[0]
    OEW = MTOW - results.get_val('Weight|payload')[0]
    W_batt = results.get_val('Weight|battery')[0]
    W_empty = OEW - W_batt
    P_cruise = results.get_val('Power|segment_5', 'kW')[0]
    DL_hover = results.get_val('DiskLoading|LiftRotor|segment_2', 'Pa')[0]
    L_by_D_cruise = MTOW * 9.799124194 * cruise_speed / (P_cruise * 1000)

    print(f'MTOW: published= 3275.387832; calculated= {MTOW}; Difference= {np.round((MTOW-3275.387832)/3275.387832*100, 2)}%')
    print(f'OEW: published= 2726.995104; calculated= {OEW}; Difference= {np.round((OEW-2726.995104)/2726.995104*100, 2)}%')
    print(f'Battery weight: published= 790.157264; calculated= {W_batt}; Difference= {np.round((W_batt-790.157264)/790.157264*100, 2)}%')
    print(f'Empty weight: published= 1936.83784; calculated= {W_empty}; Difference= {np.round((W_empty-1936.83784)/1936.83784*100, 2)}%')
    print('---------------------------------------------------------------------------------------------')
    print(f'DL_hover: published= 143.545431; calculated= {DL_hover}; Difference= {np.round((DL_hover-143.545431)/143.545431*100, 2)}%')
    print(f'P_cruise: published= 259.0401789; calculated= {P_cruise}; Difference= {np.round((P_cruise-259.0401789)/259.0401789*100, 2)}%')
    print(f'L_by_D_cruise: published= 5.8; calculated= {L_by_D_cruise}; Difference= {np.round((L_by_D_cruise-5.8)/5.8*100, 2)}%')
    print('---------------------------------------------------------------------------------------------')
    print('Parasite drag coeff non hubs = ', vehicle1.Cd0)
    print(f"LiftRotor|HoverClimb|RPM = {results.get_val('LiftRotor|HoverClimb|RPM')[0]}")
    print(f"LiftRotor|HoverClimb|thrust_residual_square = {results.get_val('LiftRotor|HoverClimb|thrust_residual_square')[0]}")
    print('Weight|residual = ', results.get_val('Weight|residual')[0])

if create_vehicle2:
    vehicle2 = LiftPlusCruiseEVTOL(tf_structure=0.891, tf_propulsion=1.0, tf_equipment=0.891)
    vehicle2.add_component(kind='battery', density=400.0, efficiency=1.0, max_discharge=1.0)
    vehicle2.add_component(kind='fuselage', length=30.0 * 0.3048, max_diameter=6.12707 * 0.3048, number_of_passenger=6, payload_per_pax=payload_per_pax)
    vehicle2.add_component(kind='wing', area=210.27814 * 0.3048**2, aspect_ratio=12.12761, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0)
    vehicle2.add_component(kind='horizontal_tail', area=39.51120 * 0.3048**2, aspect_ratio=4.30363, taper_ratio=0.6, max_root_thickness=0.15 * 3.78750 * 0.3048, thickness_to_chord_ratio=0.15)
    vehicle2.add_component(kind='vertical_tail', area=27.34325 * 0.3048**2, aspect_ratio=1.17990, max_root_thickness=0.11500 * 9.73719 * 0.3048, sweep_angle=35, thickness_to_chord_ratio=(0.115 + 0.14 + 0.15 + 0.15 + 0.115) / 5)
    vehicle2.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3048, ultimate_load_factor=5.7)

    r_lift_rotor = 5.0 * 0.3048
    r_to_R_list1 = [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
    c_to_R_list1 = [0.548288, 0.530863, 0.513438, 0.496013, 0.478588, 0.461163, 0.443738, 0.426313]
    w_to_R_list1 = [2 * (0.259375 - 0.21), 0.358125 - 0.259375, 0.456875 - 0.358125, 0.555625 - 0.456875, 0.654375 - 0.555625, 0.753125 - 0.654375, 0.851875 - 0.753125, 0.950625 - 0.851875]
    pitch_list1 = np.array([7.359375, 5.878125, 4.396875, 2.915625, 1.434375, -0.046875, -1.528125, -3.009375])
    global_twist1 = 15.0
    mean_c_to_R1 = np.sum(c_to_R_list1) / 8

    r_propeller = 4.5 * 0.3048
    r_to_R_list2 = [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
    c_to_R_list2 = [0.500656, 0.484769, 0.468881, 0.452994, 0.437106, 0.421219, 0.405331, 0.389444]
    w_to_R_list2 = [2 * (0.259375 - 0.21), 0.358125 - 0.259375, 0.456875 - 0.358125, 0.555625 - 0.456875, 0.654375 - 0.555625, 0.753125 - 0.654375, 0.851875 - 0.753125, 0.950625 - 0.851875]
    pitch_list2 = np.array([17.591875, 14.135625, 10.679375, 7.223125, 3.766875, 0.310625, -3.145625, -6.601875])
    global_twist2 = 15.0
    mean_c_to_R2 = np.sum(c_to_R_list2) / 8

    vehicle2.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_LPC',
                           radius=r_lift_rotor, hub_radius=0.21 * r_lift_rotor, mean_c_to_R=mean_c_to_R1, r_to_R_list=r_to_R_list1, c_to_R_list=c_to_R_list1, w_to_R_list=w_to_R_list1,
                           pitch_list=pitch_list1, global_twist=global_twist1, hub_length=0.3048, hub_max_diameter=2.25 * 0.3048, figure_of_merit=0.75)
    vehicle2.add_component(kind='propeller', n_propeller=1, n_blade=6, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_LPC',
                           radius=r_propeller, hub_radius=0.21 * r_propeller, mean_c_to_R=mean_c_to_R2, r_to_R_list=r_to_R_list2, c_to_R_list=c_to_R_list2, w_to_R_list=w_to_R_list2,
                           pitch_list=pitch_list2, global_twist=global_twist2, hub_length=0.3048, hub_max_diameter=1.5 * 0.3048,
                           RPM={'climb': 400.0, 'cruise': 370.0}, figure_of_merit=0.75, Cd0=0.0089)

    vehicle2.add_component(kind='boom', represented_as='fuselage', length=16.0 * 0.3048, max_diameter=0.3048, number_of_booms=4)
    # vehicle2.print_info()

    # Analysis
    analysis2 = WeightAnalysis(vehicle=vehicle2, mission=mission, fidelity={'aero': 1, 'hover_climb': 1}, sizing_mode=True, solved_by='nonlinear_solver')
    results = analysis2.evaluate(record=True, value_guess={'hover_climb_RPM': 1042.737578, 'mtow': 4000.0})

    MTOW = results.get_val('Weight|takeoff')[0]
    OEW = MTOW - results.get_val('Weight|payload')[0]
    W_batt = results.get_val('Weight|battery')[0]
    W_empty = OEW - W_batt
    P_cruise = results.get_val('Power|segment_5', 'kW')[0]
    DL_hover = results.get_val('DiskLoading|LiftRotor|segment_2', 'Pa')[0]
    L_by_D_cruise = MTOW * 9.799124194 * cruise_speed / (P_cruise * 1000)

    print(f'MTOW: published= 4300.959344; calculated= {MTOW}; Difference= {np.round((MTOW-4300.959344)/4300.959344*100, 2)}%')
    print(f'OEW: published= 3753.020208; calculated= {OEW}; Difference= {np.round((OEW-3753.020208)/3753.020208*100, 2)}%')
    print(f'Battery weight: published= 933.492336; calculated= {W_batt}; Difference= {np.round((W_batt-933.492336)/933.492336*100, 2)}%')
    print(f'Empty weight: published= 2819.527872; calculated= {W_empty}; Difference= {np.round((W_empty-2819.527872)/2819.527872*100, 2)}%')
    print('---------------------------------------------------------------------------------------------')
    print(f'DL_hover: published= 722.5120025; calculated= {DL_hover}; Difference= {np.round((DL_hover-722.5120025)/722.5120025*100, 2)}%')
    print(f'P_cruise: published= 227.7326523; calculated= {P_cruise}; Difference= {np.round((P_cruise-227.7326523)/227.7326523*100, 2)}%')
    print(f'L_by_D_cruise: published= 7.9; calculated= {L_by_D_cruise}; Difference= {np.round((L_by_D_cruise-7.9)/7.9*100, 2)}%')
    print('---------------------------------------------------------------------------------------------')
    print('Parasite drag coeff non hubs = ', vehicle2.Cd0)
    print(f"LiftRotor|HoverClimb|RPM = {results.get_val('LiftRotor|HoverClimb|RPM')[0]}")
    print(f"LiftRotor|HoverClimb|thrust_residual_square = {results.get_val('LiftRotor|HoverClimb|thrust_residual_square')[0]}")
    print('Weight|residual = ', results.get_val('Weight|residual')[0])
