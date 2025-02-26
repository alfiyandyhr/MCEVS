from MCEVS.Missions.Container import Mission

def StandardMissionProfile(mission_range, cruise_speed):

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

	return mission

def UberMissionProfile(v_stall=40.0):
	mission_range = 96560.6 # m
	cruise_speed = 67.056 # m/s

	# Transition + Climb
	TC_duration = 76.2/2.54 # s
	TC_distance_X = (1.2*v_stall/2) * TC_duration

	# Departure Terminal Procedures
	DTP_duration = 60.0 # s; one minute
	DTP_distance = 1.2*v_stall * DTP_duration

	# Accel + Climb
	AC_duration = 365.76/2.54 # s
	AC_distance_X = (1.2*v_stall + cruise_speed) * AC_duration

	# Decel + Descent
	DD_duration = 365.76/2.54 # s
	DD_distance_X = (cruise_speed + 1.2*v_stall) * DD_duration

	# Arrival Terminal Procedures
	ATP_duration = 60.0 # s; one minute
	ATP_distance = 1.2*v_stall * ATP_duration

	# Transition + Descent
	TD_duration = 76.2/1.524 # s
	TD_distance_X = (1.2*v_stall/2) * TC_duration

	# Cruise
	cruise_range = mission_range - (TC_distance_X + DTP_distance + AC_distance_X + DD_distance_X + ATP_distance + TD_distance_X)
	# print(f'cruise_range = {cruise_range/1000} km')
	# print(f'transition_climb_range = {TC_distance_X/1000} km')
	# print(f'departure_terminal_procedure_range = {DTP_distance/1000} km')
	# print(f'accelerated_climb_range = {AC_distance_X/1000} km')
	# print(f'decelerated_descent_range = {DD_distance_X/1000} km')
	# print(f'arrival_terminal_procedure_range = {ATP_distance/1000} km')
	# print(f'transition_descent_range = {TD_distance_X/1000} km')

	# print(f'departure_terminal_procedure_duration = {DTP_duration/60} min')
	# print(f'arrival_terminal_procedure_duration = {ATP_duration/60} min')

	mission = Mission()
	mission.add_segment(name='Hover Climb', kind='HoverClimbConstantAcceleration', final_speed=2.54, distance=15.24, n_discrete=10)
	mission.add_segment(name='Transition + Climb', kind='ClimbConstantVyConstantAx', distance_Y=76.2, speed_Y=2.54, final_speed_X=1.2*v_stall, n_discrete=10)
	mission.add_segment(name='Departure Terminal Procedures', kind='CruiseConstantSpeed', speed=1.2*v_stall, duration=60.0, n_discrete=5)
	mission.add_segment(name='Accel + Climb', kind='ClimbConstantVyConstantAx', distance_Y=365.76, speed_Y=2.54, final_speed_X=cruise_speed, n_discrete=10)
	mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, n_discrete=5)
	mission.add_segment(name='Decel + Descent', kind='DescentConstantVyConstantAx', distance_Y=365.76, speed_Y=2.54, final_speed_X=1.2*v_stall, n_discrete=10)
	mission.add_segment(name='Arrival Terminal Procedures', kind='CruiseConstantSpeed', speed=1.2*v_stall, duration=60.0, n_discrete=5)
	mission.add_segment(name='Transition + Descent', kind='DescentConstantVyConstantAx', distance_Y=76.2, speed_Y=1.524, final_speed_X=0.0, n_discrete=10)
	mission.add_segment(name='Hover Descent', kind='HoverDescentConstantDeceleration', initial_speed=1.524, final_speed=0.0, distance=15.24, n_discrete=10)

	return mission

def SimplifiedUberMissionProfile(mission_range, cruise_speed):

	# Accel + Climb
	AC_duration = 304.8/2.54 # s
	AC_distance_X = (cruise_speed/2) * AC_duration

	# Decel + Descent
	DD_duration = 304.8/2.54 # s
	DD_distance_X = (cruise_speed/2) * DD_duration

	# Cruise
	cruise_range = mission_range - (AC_distance_X + DD_distance_X)

	# print(f'climb range = {AC_distance_X/1000} km')
	# print(f'cruise range = {cruise_range/1000} km')
	# print(f'descent range = {DD_distance_X/1000} km')

	mission = Mission()
	mission.add_segment(name='Hover Climb', kind='HoverClimbConstantAcceleration', final_speed=2.54, distance=152.4, n_discrete=10)
	mission.add_segment(name='Accel + Climb', kind='ClimbConstantVyConstantAx', distance_Y=304.8, speed_Y=2.54, final_speed_X=cruise_speed, n_discrete=10)
	mission.add_segment(name='Mission Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=cruise_range, n_discrete=5)
	mission.add_segment(name='Decel + Descent', kind='DescentConstantVyConstantAx', distance_Y=304.8, speed_Y=2.54, final_speed_X=0.0, n_discrete=10)
	mission.add_segment(name='Hover', kind='HoverStay', duration=120.0, n_discrete=10)
	mission.add_segment(name='Reserve Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range/10, n_discrete=5)
	mission.add_segment(name='Hover Descent', kind='HoverDescentConstantDeceleration', initial_speed=1.524, final_speed=0.0, distance=152.4, n_discrete=10)

	return mission

def StandardMissionProfileOld(mission_range, cruise_speed):

	# Constant climb
	C_distance_X = (cruise_speed/2) * (304.8/2.54)

	# Constant descent
	D_distance_X = (cruise_speed/2) * (304.8/1.524)

	cruise_range = mission_range - (C_distance_X + D_distance_X)

	mission = Mission()
	mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=152.4, n_discrete=5)
	mission.add_segment(name='Transition', kind='TransitionConstantAcceleration', final_speed=cruise_speed, duration=30.0, n_discrete=5)
	mission.add_segment(name='Constant Climb', kind='ClimbConstantVyConstantVx', speed_Y=2.54, distance_Y=304.8, speed_X=cruise_speed, n_discrete=5)
	mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=cruise_range, n_discrete=5)
	mission.add_segment(name='Constant Descent', kind='DescentConstantVyConstantVx', speed_Y=1.524, distance_Y=304.8, speed_X=cruise_speed, n_discrete=5)
	mission.add_segment(name='Transition', kind='TransitionConstantAcceleration', final_speed=0.0, duration=30.0, n_discrete=10)
	mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=152.4, n_discrete=5)

	return mission

def ConstantHoverAndCruiseMissionProfile(mission_range, cruise_speed):

	mission = Mission()
	mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=152.4, n_discrete=10)
	mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', speed=cruise_speed, distance=mission_range, n_discrete=5)
	mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=152.4, n_discrete=10)

	return mission


















