from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
import warnings

analyze_ref_vehicle = False
optimize_ref_vehicle = True

# Fixed mission requirement
mission_range = 60 * 1609.344  # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s

# Standard mission
mission = StandardMissionProfile(mission_range, cruise_speed)

solution_fidelity = {'aero': 1, 'hover_climb': 0}

mtow_guess = 4000.0

if analyze_ref_vehicle:

    # Standard vehicle
    design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
    operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
    vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

    # Weight analysis
    analysis = WeightAnalysis(vehicle, mission, solution_fidelity, sizing_mode=True, solved_by='optimization')
    res = analysis.evaluate(record=False, mtow_guess=mtow_guess)

    # Bookkeeping results
    results = {}

    # Mission requirements
    results['mission_range'] = mission.segments[2].distance / 1000.0 	# km
    results['cruise_speed'] = mission.segments[2].speed * 3.6  # km/h
    results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]

    # Design objective, variables, and constraints
    results['Weight|takeoff'] = res.get_val('Weight|takeoff', 'kg')[0]
    results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
    results['LiftRotor|Cruise|RPM'] = res.get_val('LiftRotor|Cruise|RPM', 'rpm')[0]
    results['LiftRotor|Cruise|CT/sigma'] = res.get_val('LiftRotor|Cruise|thrust_coefficient')[0] / vehicle.lift_rotor.solidity
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

    print(results)

if optimize_ref_vehicle:

    mtow_guess = 4000.0

    # Standard vehicle
    design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
    operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0, 'cruise': 450.0}}
    tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}
    vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

    # Standard optimization
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        results = RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=True)
        print(results)
