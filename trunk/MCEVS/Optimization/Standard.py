from MCEVS.Optimization.Container import DesignProblem
import os
import sys

def RunStandardSingleObjectiveOptimization(vehicle:object, mission:object, fidelity:dict, objective:str, mtow_guess:bool, speed_as_design_var:bool, print=True):

	if not print: 
		if os.name =='posix': sys.stdout = open('/dev/null', 'w')  # Redirect stdout to /dev/null
		if os.name =='nt': sys.stdout = open(os.devnull, 'w')  # Redirect stdout to os.devnull

	for segment in mission.segments:
		if segment.kind == 'CruiseConstantSpeed':
			cruise_segment_id = segment.id

	if vehicle.configuration == 'Multirotor':

		# Design problem
		problem = DesignProblem(vehicle=vehicle,
								mission=mission,
								fidelity=fidelity,
								algorithm='gradient-based')

		if objective == 'takeoff_weight':
			problem.add_objective('Weight|takeoff', 3000.0, 'kg')
		elif objective == 'energy':
			problem.add_objective('Energy|entire_mission', 100000.0, 'W*h')
		elif objective == 'mission_time':
			problem.add_objective('Mission|total_time', 1800.0, 's')

		if fidelity['hover_climb'] == 0:
			problem.add_design_var('Weight|takeoff', 100.0, 10000.0, mtow_guess, 'kg')
			problem.add_design_var('LiftRotor|radius', 1.0, 5.0, vehicle.lift_rotor.radius, 'm')
			problem.add_design_var('LiftRotor|Cruise|RPM', 100.0, 1500.0, vehicle.lift_rotor.RPM['cruise'], 'rpm')
			if speed_as_design_var:
				problem.add_design_var(f'Mission|segment_{cruise_segment_id}|speed', 80*1000/3600, 320*1000/3600, mission.segments[cruise_segment_id-1].speed, 'm/s')

			problem.add_constraint('Weight|residual', 0.0, 0.0, 0.1, 'kg')
			problem.add_constraint('LiftRotor|Cruise|mu', 0.01, 1.0, 0.5, None)
			problem.add_constraint('LiftRotor|Cruise|thrust_coefficient', 0.0, 0.14*vehicle.lift_rotor.solidity, 0.10*vehicle.lift_rotor.solidity, None)
			problem.add_constraint('LiftRotor|HoverClimb|T_to_P', 0.0, 12.0, 10.0, 'g/W')
			problem.add_constraint('LiftRotor|Cruise|T_to_P', 0.0, 12.0, 10.0, 'g/W')
			problem.add_constraint('LiftRotor|HoverDescent|T_to_P', 0.0, 12.0, 10.0, 'g/W')

		# Optimization
		res = problem.run_optimization()

		# Reset stdout
		sys.stdout = sys.__stdout__  # Reset stdout back to the default

		# Bookkeeping results
		results = {}

		# Mission requirements
		results['mission_range'] = mission.segments[cruise_segment_id-1].distance/1000.0 	# km
		results['cruise_speed'] = res.get_val(f'Mission|segment_{cruise_segment_id}|speed', 'km/h')[0] if speed_as_design_var else mission.segments[cruise_segment_id-1].speed*3.6 # km/h
		if speed_as_design_var:
			results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]
		else:
			results['mission_time'] = (mission.segments[0].duration + mission.segments[cruise_segment_id-1].duration + mission.segments[4].duration)/3600.0 # h

		# Design objective, variables, and constraints
		results['Weight|takeoff'] = res.get_val('Weight|takeoff', 'kg')[0]
		results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
		results['LiftRotor|Cruise|RPM'] = res.get_val('LiftRotor|Cruise|RPM', 'rpm')[0]
		results['LiftRotor|Cruise|CT/sigma'] = res.get_val('LiftRotor|Cruise|thrust_coefficient')[0]/vehicle.lift_rotor.solidity
		results['LiftRotor|Cruise|mu'] = res.get_val('LiftRotor|Cruise|mu')[0]
		results['LiftRotor|HoverClimb|T_to_P'] = res.get_val('LiftRotor|HoverClimb|T_to_P')[0]
		results['LiftRotor|Cruise|T_to_P'] = res.get_val('LiftRotor|Cruise|T_to_P')[0]
		results['LiftRotor|HoverDescent|T_to_P'] = res.get_val('LiftRotor|HoverDescent|T_to_P')[0]
		results['Weight|residual'] = res.get_val('Weight|residual', 'kg')[0]

		# Other importants results
		results['Weight|payload'] = res.get_val('Weight|payload', 'kg')[0]
		results['Weight|battery'] = res.get_val('Weight|battery', 'kg')[0]
		results['Weight|propulsion'] = res.get_val('Weight|propulsion', 'kg')[0]
		results['Weight|structure'] = res.get_val('Weight|structure', 'kg')[0]
		results['Weight|equipment'] = res.get_val('Weight|equipment', 'kg')[0]
		results['Power|segment_1'] = res.get_val('Power|segment_1', 'kW')[0]
		results['Power|segment_3'] = res.get_val('Power|segment_3', 'kW')[0]
		results['Power|segment_5'] = res.get_val('Power|segment_5', 'kW')[0]
		results['DiskLoading|LiftRotor|segment_1'] = res.get_val('DiskLoading|LiftRotor|segment_1', 'N/m**2')[0]
		results['DiskLoading|LiftRotor|segment_3'] = res.get_val('DiskLoading|LiftRotor|segment_3', 'N/m**2')[0]
		results['DiskLoading|LiftRotor|segment_5'] = res.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2')[0]
		results['Energy|entire_mission'] = res.get_val('Energy|entire_mission', 'kW*h')[0]

		# Aerodynamics at cruise
		results['Aero|Cruise|f_total_non_hub'] = vehicle.f_total_non_hub['cruise']
		results['Aero|Cruise|f_total'] = res.get_val('Aero|Cruise|f_total', 'm**2')[0]
		results['Aero|Cruise|total_drag'] = res.get_val('Aero|Cruise|total_drag', 'N')[0]

	elif vehicle.configuration == 'LiftPlusCruise':
		
		# Design problem
		problem = DesignProblem(vehicle=vehicle,
								mission=mission,
								fidelity=fidelity,
								algorithm='gradient-based')

		if objective == 'takeoff_weight':
			problem.add_objective('Weight|takeoff', 3000.0, 'kg')
		elif objective == 'energy':
			problem.add_objective('Energy|entire_mission', 100000.0, 'W*h')
		elif objective == 'mission_time':
			problem.add_objective('Mission|total_time', 1800.0, 's')

		if fidelity['hover_climb'] == 0:
			problem.add_design_var('Weight|takeoff', 100.0, 10000.0, mtow_guess, 'kg')
			problem.add_design_var('Wing|area', 6.0, 30.0, vehicle.wing.area, 'm**2')
			problem.add_design_var('Wing|aspect_ratio', 4.0, 12.0, vehicle.wing.aspect_ratio, None)
			problem.add_design_var('LiftRotor|radius', 0.8, 2.5, vehicle.lift_rotor.radius, 'm')
			problem.add_design_var('Propeller|radius', 0.8, 2.1, vehicle.propeller.radius, 'm')
			problem.add_design_var('Propeller|Cruise|RPM', 100.0, 1500.0, vehicle.propeller.RPM['cruise'], 'rpm')
			if speed_as_design_var:
				problem.add_design_var(f'Mission|segment_{cruise_segment_id}|speed', 80*1000/3600, 320*1000/3600, mission.segments[cruise_segment_id-1].speed, 'm/s')

			problem.add_constraint('Weight|residual', 0.0, 0.0, 0.1, 'kg')
			problem.add_constraint('Aero|Cruise|CL', 0.0, 0.9, 0.5, None)
			problem.add_constraint('Propeller|Cruise|J', 0.01, 3.0, 1.0, None)
			problem.add_constraint('Propeller|Cruise|thrust_coefficient', 0.0, 0.14*vehicle.propeller.solidity, 0.10*vehicle.propeller.solidity, None)
			problem.add_constraint('LiftRotor|HoverClimb|T_to_P', 0.0, 12.0, 10.0, 'g/W')
			problem.add_constraint('Propeller|Cruise|T_to_P', 0.0, 12.0, 10.0, 'g/W')
			problem.add_constraint('LiftRotor|HoverDescent|T_to_P', 0.0, 12.0, 10.0, 'g/W')
			problem.add_constraint('LiftRotor|clearance_constraint', -1000.0, 0.0, 1.0, 'm')

		# Optimization
		res = problem.run_optimization()

		# Reset stdout
		sys.stdout = sys.__stdout__  # Reset stdout back to the default
		# Bookkeeping results
		results = {}		

		# Mission requirements
		results['mission_range'] = mission.segments[cruise_segment_id-1].distance/1000.0	# km
		results['cruise_speed'] = res.get_val(f'Mission|segment_{cruise_segment_id}|speed', 'km/h')[0] if speed_as_design_var else mission.segments[cruise_segment_id-1].speed*3.6 # km/h
		if speed_as_design_var:
			results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]
		else:
			results['mission_time'] = (mission.segments[0].duration + mission.segments[cruise_segment_id-1].duration + mission.segments[4].duration)/3600.0 # h

		# Design objective, variables, and constraints
		results['Weight|takeoff'] = res.get_val('Weight|takeoff', 'kg')[0]
		results['Wing|area'] = res.get_val('Wing|area', 'm**2')[0]
		results['Wing|aspect_ratio'] = res.get_val('Wing|aspect_ratio', None)[0]
		results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
		results['Propeller|radius'] = res.get_val('Propeller|radius', 'm')[0]
		results['Propeller|Cruise|RPM'] = res.get_val('Propeller|Cruise|RPM', 'rpm')[0]
		results['Aero|Cruise|CL'] = res.get_val('Aero|Cruise|CL', None)[0]
		results['Propeller|Cruise|CT/sigma'] = res.get_val('Propeller|Cruise|thrust_coefficient')[0]/vehicle.propeller.solidity
		results['Propeller|Cruise|J'] = res.get_val('Propeller|Cruise|J')[0]
		results['LiftRotor|HoverClimb|T_to_P'] = res.get_val('LiftRotor|HoverClimb|T_to_P')[0]
		results['Propeller|Cruise|T_to_P'] = res.get_val('Propeller|Cruise|T_to_P')[0]
		results['LiftRotor|HoverDescent|T_to_P'] = res.get_val('LiftRotor|HoverDescent|T_to_P')[0]
		results['LiftRotor|clearance_constraint'] = res.get_val('LiftRotor|clearance_constraint')[0]
		results['Weight|residual'] = res.get_val('Weight|residual', 'kg')[0]

		# Other importants results
		results['Weight|payload'] = res.get_val('Weight|payload', 'kg')[0]
		results['Weight|battery'] = res.get_val('Weight|battery', 'kg')[0]
		results['Weight|propulsion'] = res.get_val('Weight|propulsion', 'kg')[0]
		results['Weight|structure'] = res.get_val('Weight|structure', 'kg')[0]
		results['Weight|equipment'] = res.get_val('Weight|equipment', 'kg')[0]
		results['Power|segment_1'] = res.get_val('Power|segment_1', 'kW')[0]
		results['Power|segment_3'] = res.get_val('Power|segment_3', 'kW')[0]
		results['Power|segment_5'] = res.get_val('Power|segment_5', 'kW')[0]
		results['DiskLoading|LiftRotor|segment_1'] = res.get_val('DiskLoading|LiftRotor|segment_1', 'N/m**2')[0]
		results['DiskLoading|Propeller|segment_3'] = res.get_val('DiskLoading|Propeller|segment_3', 'N/m**2')[0]
		results['DiskLoading|LiftRotor|segment_5'] = res.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2')[0]
		results['Energy|entire_mission'] = res.get_val('Energy|entire_mission', 'kW*h')[0]

		# Aerodynamics at cruise
		results['Aero|Cruise|f_total_non_hub_non_wing'] = vehicle.f_total_non_hub_non_wing['cruise']
		results['Aero|Cruise|Cd0_wing'] = vehicle.wing.Cd0['cruise']
		results['Aero|Cruise|f_total'] = res.get_val('Aero|Cruise|f_total', 'm**2')[0]
		results['Aero|Cruise|total_drag'] = res.get_val('Aero|Cruise|total_drag', 'N')[0]

	return results

